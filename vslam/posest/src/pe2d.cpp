/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <posest/pe2d.h>
#include <posest/planarSFM.h>
#include <highgui.h>
//#include <features_2d/draw.h>
#include <posest/pnp_ransac.h>
#include <posest/cxeigen.hpp>

#include <iostream>
#include <algorithm>

using namespace cv;

static void camParams2Mat(const fc::CamParams& params, Mat& intrinsics)
{
  intrinsics = Mat::eye(3, 3, CV_32F);
  intrinsics.at<float> (0, 0) = params.fx;
  intrinsics.at<float> (1, 1) = params.fy;
  intrinsics.at<float> (0, 2) = params.cx;
  intrinsics.at<float> (1, 2) = params.cy;
};

namespace pe
{
void drawMatches(const Mat& img, const vector<KeyPoint>& kpts1, const vector<KeyPoint>& kpts2,
                 const vector<cv::DMatch>& matches, Mat& display)
{
  img.copyTo(display);
  for (size_t i = 0; i < matches.size(); i++)
  {
    circle(display, kpts1[matches[i].queryIdx].pt, 3, CV_RGB(255, 0, 0));
    line(display, kpts1[matches[i].queryIdx].pt, kpts2[matches[i].trainIdx].pt, CV_RGB(0, 0, 255));
    line(display, kpts1[matches[i].queryIdx].pt, kpts2[matches[i].trainIdx].pt, CV_RGB(0, 0, 255));
  }
}
;

void extractPnPData(const fc::Frame& frame1, const fc::Frame& frame2, const std::vector<cv::DMatch> &matches, vector<
    cv::Point2f>& imagePoints, vector<cv::Point3f>& objectPoints)
{
  std::cout << "extractPnPData: frame1.pts.size() = " << frame1.pts.size() << std::endl;
//  float zsum = 0, zsum2 = 0;
  for (size_t i = 0; i < matches.size(); i++)
  {
    int i1 = matches[i].queryIdx;
    int i2 = matches[i].trainIdx;

    if (frame1.goodPts[i1] == false)
      continue;

    const Eigen::Vector4d& p = frame1.pts[i1];

    //printf("i1 = %d i2 = %d\n", i1, i2);
    //printf("%f %f %f\n", p(0), p(1), p(2));
    objectPoints.push_back(Point3f(p(0), p(1), p(2)));
    imagePoints.push_back(frame2.kpts[i2].pt);

//    zsum += p(2);
//    zsum2 += p(2)*p(2);
  }

  std::cout << "extractPnPData: good points " << imagePoints.size() << std::endl;

//  int count = objectPoints.size();
//  printf("ExtractPnPData: mean z = %f, std z = %f\n", zsum/count, sqrt(zsum2/count - zsum*zsum/(count*count)));
};

int PoseEstimator2d::estimate(const fc::Frame& frame1, const fc::Frame& frame2)
{
  if(testMode == false)
  {
    return PoseEstimator::estimate(frame1, frame2);
  }
  else
  {
    return estimate(frame1, frame2, testMatches);
  }
}

void filterMatchesOpticalFlow(const fc::Frame& frame1, const fc::Frame& frame2, std::vector<cv::DMatch>& matches)
{
  vector<Point2f> points1, points2, points2_of;
  for(size_t i = 0; i < matches.size(); i++)
  {
    points1.push_back(frame1.kpts[matches[i].queryIdx].pt);
    points2.push_back(frame2.kpts[matches[i].trainIdx].pt);
  }

  vector<unsigned char> status;
  vector<float> err;
#if 0
  calcOpticalFlowPyrLK(frame1.img, frame2.img, points1, points2_of, status, err, cvSize(10, 10));
#else
#endif
  vector<cv::DMatch> matches_filtered;
  const float maxError = 2.0f;
  for(size_t i = 0; i < matches.size(); i++)
  {
    if(!status[i]) continue;
    if(norm(points2[matches[i].trainIdx] - points2_of[matches[i].trainIdx]) < maxError)
    {
      matches_filtered.push_back(matches[i]);
    }
  }

  matches = matches_filtered;

#if 1
  Mat display;
  pe::drawMatches(frame1.img, frame1.kpts, frame2.kpts, matches_filtered, display);
  namedWindow("1", 1);
  imshow("1", display);
  waitKey();
#endif
}

void filterMatchesByDistance(std::vector<cv::DMatch>& matches, float percentile = 0.1f)
{
  std::sort(matches.begin(), matches.end());

  // matches.resize does not compile due to absence of Match default constructor
  vector<cv::DMatch> filtered;
  for(size_t i = 0; i < (size_t)floor(matches.size()*percentile); i++)
  {
    filtered.push_back(matches[i]);
  }

  matches = filtered;
}

int PoseEstimator2d::estimate(const fc::Frame& frame1, const fc::Frame& frame2, const std::vector<cv::DMatch>& _matches)
{
  vector<cv::DMatch> matches = _matches;
  filterMatchesByDistance(matches, 0.5f);

  std::cout << "called PoseEstimator2d::estimate for frames " << frame1.frameId << " and " << frame2.frameId
        << std::endl;

  std::cout << "Number of points: " << frame1.kpts.size() << ", number of matches: " << matches.size() << std::endl;

  inliers.clear();

//  filterMatchesOpticalFlow(frame1, frame2, matches);
//  std::cout << "Number of matches after optical flow filtering: " << matches.size() << std::endl;

#if 0
  Mat display;
  vector<int> sample_indices;
  sample(matches.size(), 50, sample_indices);
  vector<Match> sample_matches;
  vectorSubset(matches, sample_indices, sample_matches);
  features_2d::drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, sample_matches, display);
  //pe::drawMatches(frame1.img, frame1.kpts, frame2.kpts, matches, display);
  namedWindow("1", 1);
  imshow("1", display);
  waitKey(0);
#endif

  vector<Point2f> image_points1, image_points2;
  matchesFromIndices(frame1.kpts, frame2.kpts, matches, image_points1, image_points2);

  vector<Point2f> full_image_points1 = image_points1; // sfm now filters the array of points, need to keep the full version
  vector<Point2f> full_image_points2 = image_points2;
  Mat rvec, tvec, intrinsics;
  camParams2Mat(frame1.cam, intrinsics);

  if (image_points1.size() < 4)
  {
    return 0;
  }

  const int minGoodPts = 30;
  const float inlierPoseReprojError = 1.0f;
  const float inlierReprojError = 1.0f;
  const float maxInlierDist = 100.0f;

  // extract all good pts for PnP solver
  vector<Point2f> imagePoints;
  vector<Point3f> objectPoints;
  extractPnPData(frame1, frame2, matches, imagePoints, objectPoints);
  if(!initialized_)
  {
    // too few correspondences with valid 3d points, run structure from motion
    //std::cout << "The number of 3d points " << imagePoints.size() << ", running SFM" << std::endl;
//    printf("number of source points: %d\n", image_points1.size());
    std::cout << "Running SFM" << std::endl;
    SFMwithSBA(intrinsics, image_points1, image_points2, rvec, tvec, 6.0);
//    printf("number of inliers: %d\n", image_points1.size());

    // normalize translation vector
    tvec = tvec/norm(tvec);

    usedMethod = SFM;
  }
  else if(imagePoints.size() > minGoodPts)
  {
    std::cout << "Running PnP" << std::endl;
//    std::cout << "The number of 3d points " << imagePoints.size() << ", running PnP" << std::endl;
    const int minIterCount = 10000;
    solvePnPRansac(objectPoints, imagePoints, intrinsics, Mat::zeros(5, 1, CV_32F), rvec, tvec, false, minIterCount, inlierPoseReprojError, minGoodPts);
    Mat _rvec, _tvec;
    rvec.convertTo(_rvec, CV_32F);
    tvec.convertTo(_tvec, CV_32F);
    rvec = _rvec;
    tvec = _tvec;

    usedMethod = PnP;
  }
  else
  {
    return 0;
  }

  std::cout << "finished pose estimation" << std::endl;

  setPose(rvec, tvec);

  Mat R;
  Rodrigues(rvec, R);
  vector<Point3f> cloud;
  vector<bool> valid;
  reprojectPoints(intrinsics, R, tvec, full_image_points1, full_image_points2, cloud, valid);

  // compute inliers for pose validation
  vector<bool> valid_pose = valid;
  filterInliers(cloud, full_image_points1, full_image_points2, R, tvec, intrinsics, inlierPoseReprojError, valid_pose);

  // compute inliers for 3d point cloud
  filterInliers(cloud, full_image_points1, full_image_points2, R, tvec, intrinsics, inlierReprojError, valid);

  float avgDist = 0;
  float avgZ = 0;
  int count = 0, count1 = 0, count2 = 0;
  for(size_t i = 0; i < cloud.size(); i++)
  {
    if(!valid_pose[i]) continue;
    const float infDist = 500.0f*norm(tvec);

    float dist2 = cloud[i].dot(cloud[i]);
    if(cvIsNaN(dist2)) continue;

    if(dist2 > infDist*infDist)
    {
      count1++;
      continue;
    }

    avgDist += dist2;
    avgZ += cloud[i].z;
    count++;

    if(cloud[i].z < 100) count2++;
  }
  avgDist = sqrt(avgDist/count)/norm(tvec);
  avgZ /= count;
  const float maxAvgDist = 50.0f;
  const float maxAvgZ = 100.0f;
//  std::cout << "The number of inliers: " << count << std::endl;
//  std::cout << "The number of inf points: " << count1 << std::endl;
//  std::cout << "The number of close points: " << count2 << std::endl;

#if 1
  if(getMethod() == SFM && !testMode && avgDist > maxAvgDist)
  {
    printf("pe::estimate: average point cloud z %f is higher than maximum acceptable %f\n", avgDist, maxAvgDist);
    return 0;
  }
#endif

  Mat R_inv, T_inv;
  R_inv = R.t();
  T_inv = -R_inv * tvec;

  // convert cloud into frame format
  assert(matches.size() == cloud.size());
  fc::Frame& _frame1 = const_cast<fc::Frame&> (frame1);
  fc::Frame& _frame2 = const_cast<fc::Frame&> (frame2);

  if(getMethod() == SFM)
  {
    _frame1.pts.resize(frame1.kpts.size());
    _frame1.goodPts.assign(frame1.kpts.size(), false);
  }
  else
  {
    assert(frame1.pts.size() == frame1.goodPts.size() == frame1.kpts.size());
  }
  _frame2.pts.resize(frame2.kpts.size());
  _frame2.goodPts.assign(frame2.kpts.size(), false);

  assert(valid.size() == cloud.size() && cloud.size() == matches.size());
  int goodCount = 0;
  double minz = 1e10, maxz = 0;
  vector<cv::DMatch> _inliers;

  float z1sum = 0, z2sum = 0, z1sum2 = 0, z2sum2 = 0;
  int countFiltered = 0;
  for (size_t i = 0; i < matches.size(); i++)
  {
    if (!valid[i])
      continue;

    if(cvIsNaN(cloud[i].x) || cvIsNaN(cloud[i].y) || cvIsNaN(cloud[i].z))
      continue;


    minz = MIN(minz, cloud[i].z);
    maxz = MAX(maxz, cloud[i].z);

#if 1
    const float maxDist = 100.0f;
    const float minDist = 10.0f;
    float dist = norm(cloud[i])/norm(tvec);
    if(!testMode && dist < minDist || dist > maxDist || cloud[i].z < 0)
    {
      countFiltered++;
      continue;
    }
#endif

    inliers.push_back(matches[i]);
    int i1 = matches[i].queryIdx;
    int i2 = matches[i].trainIdx;

    if(norm(cloud[i]) < 10)
    {
      _inliers.push_back(matches[i]);
    }

    if(getMethod() == SFM)
    {
      _frame1.pts[i1] = Eigen::Vector4d(cloud[i].x, cloud[i].y, cloud[i].z, 1.0);
      _frame1.goodPts[i1] = true;
    }

    _frame2.goodPts[i2] = true;

    // convert to frame 2 coordinate system
    Mat p(3, 1, CV_32F, &cloud[i]);
#if 0
    Mat r = R_inv * p + T_inv;
#else
    Mat r = R * p + tvec;
#endif

    Point3f _r = *r.ptr<Point3f> (0);
    //printf("assigning point %d to %f,%f,%f\n", i2, _r.x, _r.y, _r.z);
    _frame2.pts[i2] = Eigen::Vector4d(_r.x, _r.y, _r.z, 1.0);
    goodCount++;

    assert(cloud[i].z >= 0 && _r.z >= 0);

    z1sum += _frame1.pts[i1](2);
    z1sum2 += _frame1.pts[i1](2)*_frame1.pts[i1](2);
    z2sum += _frame2.pts[i2](2);
    z2sum2 += _frame2.pts[i2](2)*_frame2.pts[i2](2);
  }

  if(goodCount < minGoodPts)
  {
    printf("Returning a small amount of good points: %d, minGoodPts = %d\n", goodCount, minGoodPts);
    return 0;
  }

  printf("minz = %f, maxz = %f\n", minz, maxz);
  printf("Frame1: mean z = %f, std z = %f\n", z1sum/goodCount, sqrt(z1sum2/goodCount - z1sum*z1sum/(goodCount*goodCount)));
  printf("Frame2: mean z = %f, std z = %f\n", z2sum/goodCount, sqrt(z2sum2/goodCount - z2sum*z2sum/(goodCount*goodCount)));

  std::cout << "Input number of matches " << (int)full_image_points1.size() << ", inliers " << (int)inliers.size()
      << std::endl;
  std::cout << "goodCount " << goodCount << std::endl;
  std::cout << "filtered inliers: " << countFiltered << std::endl;
  std::cout << "_inliers.size() = " << _inliers.size() << std::endl;

#if 1
  if(!frame1.img.empty())
  {
#if 1
    Mat display1, display2;
    vector<cv::DMatch> match_samples;
    vector<int> match_indices;
    sample(matches.size(), 100, match_indices);
    vectorSubset(matches, match_indices, match_samples);
    drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, match_samples, display1);
    //pe::drawMatches(frame1.img, frame1.kpts, frame2.kpts, inliers, display1);
    namedWindow("1", 1);
    imshow("1", display1);

    char buf1[1024];
    sprintf(buf1, "matches%04d.jpg", frame1.frameId);
    imwrite(buf1, display1);

#if 0
    features_2d::drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, _inliers, display2);
    namedWindow("2", 1);
    imshow("2", display2);
#endif
#endif

    /// @todo Don't have cv::drawKeypoints, cv::drawMatches might not be used right here
    Mat img_matches;
    vector<cv::DMatch> inlier_sample;
    vector<int> inlier_indices;
    sample(inliers.size(), 50, inlier_indices);
    vectorSubset(inliers, inlier_indices, inlier_sample);
#if 1
    drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, inlier_sample, img_matches);
    namedWindow("matches", 1);
    imshow("matches", img_matches);

    char buf[1024];
    sprintf(buf, "img%04d.jpg", frame1.frameId);
    imwrite(buf, img_matches);
#endif

#if 0
    namedWindow("frame1", 1);
    Mat img1;
    cv::drawKeypoints(frame1.img, frame1.kpts, img1);
    imshow("frame1", img1);

    namedWindow("frame2", 1);
    Mat img2;
    cv::drawKeypoints(frame2.img, frame2.kpts, img2);
    imshow("frame2", img2);
#endif
  }
//  waitKey(0);
#endif

  initialized_ = true;
  return goodCount;//(int)inliers.size();

};

void PoseEstimator2d::setPose(const cv::Mat& rvec, const cv::Mat& tvec)
{
  Mat R, tvec_inv;
  Rodrigues(rvec, R);

  // invert the pose for the frame
  R = R.t();
  tvec_inv = -R * tvec;
  //  cv2eigen(R, rot);
  //  cv2eigen(tvec_inv, trans);

  dumpFltMat("tvec in SetPose", tvec);
  dumpFltMat("rvec in SetPose", rvec);

//  std::cout << "---------------------" << std::endl;
//  std::cout << "translation: " << trans << std::endl;
//  std::cout << "---------------------" << std::endl;
};

};
