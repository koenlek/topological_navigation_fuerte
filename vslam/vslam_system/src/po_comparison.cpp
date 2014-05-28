/*
 * po_comparison.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: alex
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <vslam_system/vslam.h>
#include <posest/pe3d.h>
#include <posest/peh.h>
#include <sba/sba.h>
#include <sba/sba_file_io.h>
#include <frame_common/frame.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>

#include <posest/pe3d.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace sba;
using namespace frame_common;
using namespace cv;
using namespace vslam;

// Names of left and right files in directory (with wildcards)
char *lreg, *rreg, *dreg;

// Filters for scandir
int getleft(struct dirent const *entry)
{
  if (!fnmatch(lreg, entry->d_name, 0))
    return 1;
  return 0;
}

int getright(struct dirent const *entry)
{
  if (!fnmatch(rreg, entry->d_name, 0))
    return 1;
  return 0;
}

int getidir(struct dirent const *entry)
{
  if (!fnmatch(dreg, entry->d_name, 0))
    return 1;
  return 0;
}


void project3dPoint(const Point3f& point, const Mat& rvec, const Mat& tvec, Point3f& modif_point)
{
  Mat R(3, 3, CV_64FC1);
  Rodrigues(rvec, R);
  modif_point.x = R.at<double> (0, 0) * point.x + R.at<double> (0, 1) * point.y + R.at<double> (0, 2)
     * point.z + tvec.at<double> (0, 0);
  modif_point.y = R.at<double> (1, 0) * point.x + R.at<double> (1, 1) * point.y + R.at<double> (1, 2)
     * point.z + tvec.at<double> (1, 0);
  modif_point.z = R.at<double> (2, 0) * point.x + R.at<double> (2, 1) * point.y + R.at<double> (2, 2)
     * point.z + tvec.at<double> (2, 0);
}


void invert(Mat& rvec, Mat& tvec)
{
  Mat R, RInv;
  Rodrigues(rvec,R);
  RInv = R.inv();
  Rodrigues(RInv, rvec);
  tvec = RInv*tvec;
  tvec = tvec*(-1);
}

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    printf("Args are: <param file> <image dir> <left image file template> <right image file template>\n");
    exit(0);
  }

  // get camera parameters, in the form: fx fy cx cy tx
  fstream fstr;
  fstr.open(argv[1], fstream::in);
  if (!fstr.is_open())
  {
    printf("Can't open camera file %s\n", argv[1]);
    exit(0);
  }
  CamParams camp;
  fstr >> camp.fx;
  fstr >> camp.fy;
  fstr >> camp.cx;
  fstr >> camp.cy;
  fstr >> camp.tx;
  Mat intrinsic = Mat::zeros(Size(3, 3), CV_64F);
  intrinsic.at<double>(0, 0) = camp.fx;
  intrinsic.at<double>(0, 2) = camp.cx;
  intrinsic.at<double>(1, 1) = camp.fy;
  intrinsic.at<double>(1, 2) = camp.cy;
  intrinsic.at<double>(2, 2) = 1.0;

  cout << "Cam params: " << camp.fx << " " << camp.fy << " " << camp.cx << " " << camp.cy << " " << camp.tx << endl;

  // set up directories
  struct dirent **lims, **rims, **dirs;
  int nlim, nrim, ndirs;
  string dname = argv[2];
  if (!dname.compare(dname.size() - 1, 1, "/")) // see if slash at end
    dname.erase(dname.size() - 1);

  string dirfn = dname.substr(dname.rfind("/") + 1);
  string tdir = dname.substr(0, dname.rfind("/") + 1);
  cout << "Top directory is " << tdir << endl;
  cout << "Search directory name is " << dirfn << endl;
  dreg = (char *)dirfn.c_str();

  ndirs = scandir(tdir.c_str(), &dirs, getidir, alphasort);
  printf("Found %d directories\n", ndirs);
  printf("%s\n", dirs[0]->d_name);

  const std::string window_name = "matches";
  cv::namedWindow(window_name, 0);
  cv::Mat display;

  int iter = 0;
  lreg = argv[3];
  rreg = argv[4];

  // loop over directories
  for (int dd = 0; dd < ndirs; dd++)
  {
    char dir[2048];
    sprintf(dir, "%s%s", tdir.c_str(), dirs[dd]->d_name);
    printf("Current directory: %s\n", dir);

    // get left/right image file names, sorted
    nlim = scandir(dir, &lims, getleft, alphasort);
    printf("Found %d left images\n", nlim);
    printf("%s\n", lims[0]->d_name);

    nrim = scandir(dir, &rims, getright, alphasort);
    printf("Found %d right images\n", nrim);
    printf("%s\n", rims[0]->d_name);

    if (nlim != nrim)
    {
      printf("Number of left/right images does not match: %d vs. %d\n", nlim, nrim);
      exit(0);
    }

    frame_common::Frame prevFrame;
    frame_common::FrameProc frameProcessor(5);
    typedef cv::CalonderDescriptorExtractor<float> Calonder;
    frameProcessor.setFrameDescriptor(new Calonder(argv[7]));

    pe::HowardStereoMatcher matcherError(0.01, 17);

    pe::PoseEstimator3d po(5000, true, 10.0, 3.0, 3.0);
    pe::PoseEstimatorH poh(5000, true, 10.0, 3.0, 3.0, 0.1, 40, 13);

    po.wx = poh.wx = 92;
    po.wy = poh.wy = 48;

    cv::TickMeter poMeter, howMeter;

    // loop over each stereo pair, adding it to the system
    for (int ii = 0; ii < nlim; iter++, ii++)
    {
      bool matched = false;
      // Load images
      char fn[2048];
      sprintf(fn, "%s/%s", dir, lims[ii]->d_name);
      printf("%s\n", fn);
      cv::Mat image1 = cv::imread(fn, 0);
      sprintf(fn, "%s/%s", dir, rims[ii]->d_name);
      printf("%s\n", fn);
      cv::Mat image1r = cv::imread(fn, 0);
      if (image1.rows == 0 || image1r.rows == 0)
         exit(0);

      Mat im; image1.copyTo(im);
      //bilateralFilter(im, image1, -1, 1, 1);
      Mat imr; image1r.copyTo(imr);
      //bilateralFilter(imr, image1r, -1, 1, 1);
      frame_common::Frame frame;
      poMeter.start();
      howMeter.start();
      frameProcessor.setStereoFrame(frame, image1, image1r, 0, false);
      poMeter.stop();
      howMeter.stop();
      frame.setCamParams(camp);
      frame.frameId = ii;

      vector<DMatch> matchesError;
      vector<Point3f> opointsError;
      vector<Point2f> ipointsError;
      if (!prevFrame.img.empty())
      {
        display.create(prevFrame.img.rows, prevFrame.img.cols + frame.img.cols, CV_8UC3);
        cv::Mat left = display(cv::Rect(0, 0, prevFrame.img.cols, prevFrame.img.rows));
        cvtColor(prevFrame.img, left, CV_GRAY2BGR);
        cv::Mat right = display(cv::Rect(prevFrame.img.cols, 0, frame.img.cols, frame.img.rows));
        cvtColor(frame.img, right, CV_GRAY2BGR);
        for (size_t i = 0; i < prevFrame.kpts.size(); ++i)
        {
          cv::Point pt1 = prevFrame.kpts[i].pt;
          cv::circle(display, pt1, 3, Scalar(255));
        }
        for (size_t i = 0; i < frame.kpts.size(); ++i)
        {
          cv::Point pt1(frame.kpts[i].pt.x+prevFrame.img.cols,frame.kpts[i].pt.y);
          cv::circle(display, pt1, 3, Scalar(255));
        }

        howMeter.start();
        matched = poh.estimate(prevFrame, frame);
        cout << "Inliers = " << poh.inliers.size() << endl;
        howMeter.stop();

        cout << "Matcher error" << endl;
        Mat mask = cv::windowedMatchingMask(prevFrame.kpts, frame.kpts, 92, 48);
        vector<int> filteredInidices;
        matcherError.match(prevFrame, frame, matchesError, filteredInidices, mask);
        cout << "Matches error size = " << filteredInidices.size() << endl;

        if (matched)
        {
          for (size_t i = 0; i < poh.inliers.size(); ++i)
          {
            if (prevFrame.goodPts[poh.inliers[i].queryIdx] && frame.goodPts[poh.inliers[i].trainIdx])
            {
              cv::Point pt1(prevFrame.kpts[poh.inliers[i].queryIdx].pt.x,prevFrame.kpts[poh.inliers[i].queryIdx].pt.y);
              cv::Point pt2(frame.kpts[poh.inliers[i].trainIdx].pt.x+prevFrame.img.cols,frame.kpts[poh.inliers[i].trainIdx].pt.y);
              cv::line(display, pt1, pt2, Scalar(0, 255));
            }
          }

          cout << "Matches Error size = " << filteredInidices.size() << endl;
          for (size_t i = 0; i < filteredInidices.size(); ++i)
          {
            if (prevFrame.goodPts[matchesError[filteredInidices[i]].queryIdx] && frame.goodPts[matchesError[filteredInidices[i]].trainIdx])
            {
              ipointsError.push_back(frame.kpts[matchesError[filteredInidices[i]].trainIdx].pt);
              Eigen::Vector4d vec = prevFrame.pts[matchesError[filteredInidices[i]].queryIdx];
              Point3f op(vec(0), vec(1), vec(2));
              opointsError.push_back(op);
              //cv::line(display, frame.kpts[matchesError[filteredInidices[i]].trainIdx].pt, cv::Point(prevFrame.kpts[matchesError[filteredInidices[i]].queryIdx].pt.x+prevFrame.img.cols,
                //                          prevFrame.kpts[matchesError[filteredInidices[i]].queryIdx].pt.y), Scalar(0, 255));
            }
          }
        }
      }

      if (matched /*&& opointsError.size()*/)
      {
        Mat R(3,3, CV_64F);
        for (int j=0; j<poh.rot.cols(); ++j)
          for (int i=0; i<poh.rot.rows(); ++i)
            R.at<double>(i, j) = poh.rot(i, j);
        Mat tvec(3, 1, CV_64F);
        for (int j=0; j<3; ++j)
          tvec.at<double>(0,j) = poh.trans(j);
        Mat rvec;
        Rodrigues(R, rvec);
        invert(rvec, tvec);
        vector<Point2f> projectedPoints;
        Mat distCoeffs = Mat::zeros(Size(1, 5), CV_64F);
        projectPoints(Mat(opointsError), rvec, tvec, intrinsic, distCoeffs, projectedPoints);
        float reprojectionError = 0;
        for (size_t pointInd = 0; pointInd < projectedPoints.size(); pointInd++)
        {
          float error = norm(projectedPoints[pointInd] - ipointsError[pointInd]);
          if (error > reprojectionError)
            reprojectionError = error;
        }
        cout << endl << "Howard pose estimator" << endl;
        cout << "Reprojection error = " << reprojectionError << endl;
        cout << "Inliers size = " << poh.inliers.size() << endl;
        Mat Rinv;
        Rodrigues(rvec, Rinv);
        cout << Rinv << endl << tvec << endl;


        poMeter.start();
        po.estimate(prevFrame, frame);
        poMeter.stop();
//        for (size_t i = 0; i < po.matches.size(); ++i)
//        {
//           if (prevFrame.goodPts[po.matches[i].queryIdx] && frame.goodPts[po.matches[i].trainIdx])
//           {
//             cv::Point pt1(prevFrame.kpts[po.matches[i].queryIdx].pt.x,prevFrame.kpts[po.matches[i].queryIdx].pt.y);
//             cv::Point pt2(frame.kpts[po.matches[i].trainIdx].pt.x+prevFrame.img.cols,frame.kpts[po.matches[i].trainIdx].pt.y);
//             //cv::line(display, pt1, pt2, Scalar(0,0,255));
//           }
//        }
        Mat poR;
        R.copyTo(poR);
        for (int j=0; j<po.rot.cols(); ++j)
          for (int i=0; i<po.rot.rows(); ++i)
            poR.at<double>(i, j) = po.rot(i, j);
        Mat t;
        tvec.copyTo(t);
        for (int j=0; j<3; ++j)
          t.at<double>(0,j) = po.trans(j);
        Mat r;
        Rodrigues(poR, r);
        invert(r, t);
        projectPoints(Mat(opointsError), r, t, intrinsic, Mat::zeros(Size(1, 5), CV_64F), projectedPoints);
        float reprojectionError3d = 0;
        for (size_t pointInd = 0; pointInd < projectedPoints.size(); pointInd++)
        {
          float error = norm(projectedPoints[pointInd] - ipointsError[pointInd]);
          if (error > reprojectionError3d)
            reprojectionError3d = error;
        }
        cout << endl << "3d pose estimator" << endl;
        cout << "Reprojection error = " << reprojectionError3d << endl;
        cout << "Inliers size = " << po.inliers.size() << ", matches = " << po.matches.size() << endl;
        Mat RR;
        Rodrigues(r, RR);
        cout << RR << endl << t << endl;

        cout << "Differences:" << endl;
        Mat Rdiff = Rinv - RR;
        float rdiff = norm(Rdiff, NORM_INF);
        cout << Rdiff << endl;
        cout << "R diff = " << rdiff << endl;
        cout << "tdiff = " << norm(tvec-t, NORM_INF) << endl;
        ofstream f;
        f.open("statistic.txt", fstream::app);
        f << ii << " " << rdiff << " " <<  norm(tvec-t, NORM_INF) << " " <<  reprojectionError << " " << reprojectionError3d << " " << poh.inliers.size() << " " << po.inliers.size() << endl;
        f.close();


        cv::imshow(window_name, display);
        cout << "How time = " << howMeter.getTimeMilli() / (float)(ii+1) << endl;
        cout << "Po time = " << poMeter.getTimeMilli() / (float)(ii+1) << endl;
        char key;
        while (true)
        {
          key = cv::waitKey(1);
          if (key == 32)
          {
            break;
          }
          else if (key == 27)
            return -1;
        }
        //cv::waitKey(30);

      }
      prevFrame = frame;
    }
  }
  return 0;
}


