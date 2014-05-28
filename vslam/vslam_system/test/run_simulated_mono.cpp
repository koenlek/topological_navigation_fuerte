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


//
// runs a sequence of stereo images into an SBA engine
// visualize in rviz
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <vslam_system/vo.h>
#include <vslam_system/place_recognizer.h>
#include <posest/pe3d.h>
#include <sba/sba.h>
#include <frame_common/frame.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>

#include <opencv/highgui.h>

#include <posest/test/simulated.h>
#include <posest/planarSFM.h>

using namespace std;
using namespace sba;
using namespace vslam;
using namespace frame_common;
using namespace Eigen;

// elapsed time in milliseconds
#include <sys/time.h>
static double mstime()
{
  timeval tv;
  gettimeofday(&tv,NULL);
  long long ts = tv.tv_sec;
  ts *= 1000000;
  ts += tv.tv_usec;
  return (double)ts*.001;
}


// draw the graph on rviz
void
drawgraph(SysSBA &sba, SysSPA &spa, ros::Publisher &cam_pub, ros::Publisher &pt_pub, int dec,
          ros::Publisher &cst_pub, ros::Publisher &link_pub)
{
  visualization_msgs::Marker cammark, ptmark, cstmark;
  cammark.header.frame_id = "/pgraph";
  cammark.header.stamp = ros::Time();
  cammark.ns = "pgraph";
  cammark.id = 0;
  cammark.action = visualization_msgs::Marker::ADD;
  cammark.pose.position.x = 0;
  cammark.pose.position.y = 0;
  cammark.pose.position.z = 0;
  cammark.pose.orientation.x = 0.0;
  cammark.pose.orientation.y = 0.0;
  cammark.pose.orientation.z = 0.0;
  cammark.pose.orientation.w = 1.0;
  cammark.scale.x = 0.02;
  cammark.scale.y = 0.02;
  cammark.scale.z = 0.02;
  cammark.color.r = 0.0f;
  cammark.color.g = 1.0f;
  cammark.color.b = 1.0f;
  cammark.color.a = 1.0f;
  cammark.lifetime = ros::Duration();
  cammark.type = visualization_msgs::Marker::LINE_LIST;

  ptmark = cammark;
  ptmark.color.r = 1.0f;
  ptmark.color.g = 0.0f;
  ptmark.color.b = 0.0f;
  ptmark.color.a = 0.5f;
  ptmark.scale.x = 0.2;
  ptmark.scale.y = 0.2;
  ptmark.scale.z = 0.2;
  ptmark.type = visualization_msgs::Marker::POINTS;

  cstmark = cammark;
  cstmark.color.r = 1.0f;
  cstmark.color.g = 1.0f;
  cstmark.color.b = 0.0f;
  cstmark.color.a = 1.0f;
  cstmark.scale.x = 0.03;
  cstmark.scale.y = 0.03;
  cstmark.scale.z = 0.03;
  cstmark.type = visualization_msgs::Marker::LINE_LIST;


  // draw points, decimated
  int npts = sba.tracks.size();

  cout << "Number of points to draw: " << npts << endl;
  if (npts <= 0) return;


  ptmark.points.resize(npts/dec+1);
  float zsum = 0, zsum2 = 0;
  for (int i=0, ii=0; i<npts; i+=dec, ii++)
    {
      Vector4d &pt = sba.tracks[i].point;
      ptmark.points[ii].x = pt(0);
      ptmark.points[ii].y = pt(2);
      ptmark.points[ii].z = -pt(1);

      zsum += pt(2);
      zsum2 += pt(2)*pt(2);

//      printf("%f %f %f\n", pt(0), pt(1), pt(2));
    }

  int count = ptmark.points.size();
  printf("mean z: %f, std z: %f\n", zsum/count, sqrt(zsum2/count - zsum*zsum/(count*count)));

  // draw cameras
  int ncams = sba.nodes.size();
  cammark.points.resize(ncams*6);
  for (int i=0, ii=0; i<ncams; i++)
    {
      Node &nd = sba.nodes[i];
      Vector3d opt;
      Matrix<double,3,4> tr;
      transformF2W(tr,nd.trans,Quaternion<double>(nd.qrot));

      printf("camera transform: trans %f %f %f, rot %f %f %f %f\n", nd.trans.x(),
             nd.trans.y(), nd.trans.z(), nd.qrot.x(), nd.qrot.y(), nd.qrot.z(), nd.qrot.w());
      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0,0.3,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0.2,0,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0.1,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();
    }

  // draw SPA constraints
  int ncons = spa.p2cons.size();
  cstmark.points.resize(ncons*6);

  for (int i=0, ii=0; i<ncons; i++)
    {
      ConP2 &con = spa.p2cons[i];
      Node &nd0 = spa.nodes[con.ndr];
      Node &nd1 = spa.nodes[con.nd1];

      Node &nd = spa.nodes[i];
      Vector3d opt;
      Matrix<double,3,4> tr;
      transformF2W(tr,nd.trans,Quaternion<double>(nd.qrot));

      cstmark.points[ii].x = nd.trans.x();
      cstmark.points[ii].y = nd.trans.z();
      cstmark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0,0.3,1);
      cstmark.points[ii].x = opt.x();
      cstmark.points[ii].y = opt.z();
      cstmark.points[ii++].z = -opt.y();

      cstmark.points[ii].x = nd.trans.x();
      cstmark.points[ii].y = nd.trans.z();
      cstmark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0.2,0,0,1);
      cstmark.points[ii].x = opt.x();
      cstmark.points[ii].y = opt.z();
      cstmark.points[ii++].z = -opt.y();

      cstmark.points[ii].x = nd.trans.x();
      cstmark.points[ii].y = nd.trans.z();
      cstmark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0.1,0,1);
      cstmark.points[ii].x = opt.x();
      cstmark.points[ii].y = opt.z();
      cstmark.points[ii++].z = -opt.y();

#if 0
      cstmark.points[ii].x = nd0.trans.x();
      cstmark.points[ii].y= nd0.trans.z();
      cstmark.points[ii++].z = -nd0.trans.y();
      cstmark.points[ii].x = nd1.trans.x();
      cstmark.points[ii].y = nd1.trans.z();
      cstmark.points[ii++].z = -nd1.trans.y();
#endif
    }

  cam_pub.publish(cammark);
  pt_pub.publish(ptmark);
  cst_pub.publish(cstmark);
  std::cout << "Visualization messages published" << std::endl;

#if 1
  cv::namedWindow("1", 1);
  cv::waitKey();
#endif
}



// main loop

// parameters
//  max distance, angle between keyframes
//  min inliers between keyframes

double maxdist = 0.1;           // meters
double maxang  = 10.0;          // degrees
int mininls    = 100;          // depends on number of points, no?
int ndi = 0;                    // current keyframe index

void generateData(std::vector<cv::Point3f>& cloud)
{
  vector<cv::Point3f> planarPoints, pointCloud;
  planarPoints.resize(500);
  pe::generatePlanarObject(planarPoints, cv::Point3f(1.0f, 0.0f, 0.0f), 1.0f);
  pointCloud.resize(500);
  pe::generate3DPointCloud(pointCloud);

  cloud = planarPoints;
//  cloud.insert(cloud.end(), pointCloud.begin(), pointCloud.end());
}


static void camParams2Mat(const fc::CamParams& params, cv::Mat& intrinsics)
{
  intrinsics = cv::Mat::eye(3, 3, CV_32F);
  intrinsics.at<float> (0, 0) = params.fx;
  intrinsics.at<float> (1, 1) = params.fy;
  intrinsics.at<float> (0, 2) = params.cx;
  intrinsics.at<float> (1, 2) = params.cy;
};

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Args are: <param file>\n");
      exit(0);
    }

  // get camera parameters, in the form: fx fy cx cy tx
  fstream fstr;
  fstr.open(argv[1],fstream::in);
  if (!fstr.is_open())
    {
      printf("Can't open camera file %s\n",argv[1]);
      exit(0);
    }
  CamParams camp;
  fstr >> camp.fx;
  fstr >> camp.fy;
  fstr >> camp.cx;
  fstr >> camp.cy;
  fstr >> camp.tx;

  cout << "Cam params: " << camp.fx << " " << camp.fy << " " << camp.cx
       << " " << camp.cy << " " << camp.tx << endl;

  cv::Mat intrinsics;
  camParams2Mat(camp, intrinsics);

  // set up structures
  cout << "Setting up frame processing..." << flush;
  FrameProc fp(30);//18);             // can be slow, setting up Calonder tree
  cout << "done" << endl;
  vector<Frame, Eigen::aligned_allocator<Frame> > frames; // stereo image frames in system

  SysSBA sba;			// SBA system
  sba.useCholmod(false);

  SysSPA spa;                   // SPA system
  int spaFrameId = -1;          // current frame of SPA system
  int ndi0 = 0;                 // current node of SPA system

  // VO processor
  voSt vo(boost::shared_ptr<pe::PoseEstimator>(new pe::PoseEstimator2d), 40,10,mininls,maxdist,maxang); // 40 frames, 10 fixed

  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  ros::NodeHandle nh ("~");
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker>("points", 0);
  ros::Publisher cam_pub = nh.advertise<visualization_msgs::Marker>("cameras", 0);
  ros::Publisher cst_pub = nh.advertise<visualization_msgs::Marker>("constraints", 0);
  ros::Publisher link_pub = nh.advertise<visualization_msgs::Marker>("links", 0);

  // for RANSAC
  srand(mstime());

  int iter = 0;

  // generate point cloud
/*  std::vector<cv::Point3f> cloud;
  generateData(cloud);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  tvec.at<float>(0, 0) = -0.5f;*/

  vo.pose_estimator_->setTestMode(true);
/*
  std::vector<f2d::Match> matches;
  for(size_t i = 0; i < cloud.size(); i++)
  {
    matches.push_back(f2d::Match(i, i, 0.0));
  }
//  vo.pe.setTestMatches(matches); // the same matches will be used for the whole test sequence
*/

  bool init = true;
  std::vector<cv::Point3f> cloud;
  pe::generateRing(cloud);
  pe::CircleCameraSimulator simulator(intrinsics, cloud);
//  for(float zcam = -1; zcam < 1; zcam += 0.1)
  for(int i = 0; i < 100; i++)
  {
//    std::cout << std::endl << "***********************************************" << std::endl;
//    std::cout << "Current camera position: z = " << zcam << std::endl << std::endl;

    // generate rvec and tvec
//    tvec.at<float>(2, 0) = -zcam;
//    rvec.at<float>(0, 0) = rvec.at<float>(0, 0) + 0.1f;

    // create a new frame
    double t0 = mstime();
    Frame f1;             // next frame
    f1.setCamParams(camp); // this sets the projection and reprojection matrices

    // set matches
//    std::vector<f2d::Match> testMatches = matches;
//    pe::addLinkNoise(testMatches, 0.3);
    std::vector<cv::DMatch> testMatches;
    std::vector<cv::KeyPoint> keypoints;
    simulator.getNextFrame(f1.kpts, testMatches);

    vo.pose_estimator_->setTestMatches(testMatches);

    // generate keypoints
//    pe::generateProjections(intrinsics, rvec, tvec, cloud, f1.kpts);

    f1.pts.resize(f1.kpts.size());
    f1.goodPts.assign(f1.kpts.size(), false);
    f1.disps.assign(f1.kpts.size(), 10);
    f1.frameId = sba.nodes.size(); // index
    f1.img = cv::Mat();   // remove the images
    f1.imgRight = cv::Mat();

/*
    if(frames.size() > 0)
    {
      std::vector<cv::Point2f> points1, points2;
      pe::keyPoints2Point2f(frames.back().kpts, points1);
      pe::keyPoints2Point2f(f1.kpts, points2);

      cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
      tvec.at<float>(2, 0) = -1.0f;

      std::vector<cv::Point3f> cloud;
      std::vector<bool> valid;

      pe::reprojectPoints(intrinsics, R, tvec, points1, points2, cloud, valid);
      float zsum = 0;
      int count = 0;
      for(size_t i = 0; i < cloud.size(); i++)
      {
        if(!valid[i]) continue;
        count++;
        zsum += cloud[i].z;
      }
      printf("run_simulated_mono: after reprojectPoints %d, mean z = %f\n", count, zsum/count);
    }
*/
    // VO
    cout << "calling vo::addFrame" << endl;
    bool ret = vo.addFrame(f1);

    // grow full SBA
    if (ret)
    {
      frames.push_back(f1);
      if(frames.size() > 1)
      {
        Frame& _fs = frames.back();
        Frame& _f0 = *(frames.end() - 2);

        if(vo.pose_estimator_->getMethod() == pe::PoseEstimator::SFM)
        {
//                assert(frames.size() == 2);
          cout << "frames.size() " << frames.size() << endl;
          frames[0].pts = (vo.frames.end() - 2)->pts;
          frames[0].goodPts = (vo.frames.end() - 2)->goodPts;
        }

        int count = 0;
        for(size_t i = 0; i < _f0.goodPts.size(); i++)
        {
          if(_f0.goodPts[i]) count++;
        }
        printf("The number of good points: %d\n", count);
      }
      //              if(frames.size() > 1)
      {
        vo.transferLatestFrame(frames,sba);
      }
    }

    if (ret)
      {
        int n = sba.nodes.size();
        int np = sba.tracks.size();

        // draw graph
        cout << "drawing with " << n << " nodes and " << np << " points..." << endl << endl;
#if 1
        if (n%2 == 0)
          drawgraph(sba,spa,cam_pub,pt_pub,1,cst_pub,link_pub); // every nth point
#endif

#if 1
        int nnsba = 10;
        if (n > 4 && n%nnsba == 0)
          {
            cout << "Running large SBA" << endl;
//            sba.doSBA(3,1.0e-4,1);
          }
#endif
      }

#ifdef VISMATCH
    // Visualize
    if (ret || 1)
      {
        drawVOtracks(image1,vo.frames,display);
        cv::imshow(window_name, display);
        cv::waitKey(10);
      }
#endif

if (!nh.ok())
  return 0;
  }
  return 0;
}
