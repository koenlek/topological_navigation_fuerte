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

#include <vslam_system/vslam.h>
#include <posest/pe3d.h>
#include <sba/sba.h>
#include <sba/sba_file_io.h>
#include <frame_common/frame.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>

#include <opencv/highgui.h>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace sba;
using namespace frame_common;
using namespace Eigen;
using namespace vslam;

// visual output of matches
#define VISMATCH

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

// Names of left and right files in directory (with wildcards)
char *lreg, *rreg, *dreg;

// Filters for scandir
int getleft(struct dirent const *entry)
{
  if (!fnmatch(lreg,entry->d_name,0))
    return 1;
  return 0;
}

int getright(struct dirent const *entry)
{
  if (!fnmatch(rreg,entry->d_name,0))
    return 1;
  return 0;
}


int getidir(struct dirent const *entry)
{
  if (!fnmatch(dreg,entry->d_name,0))
    return 1;
  return 0;
}



// draw the graph on rviz
void
drawgraph(const SysSBA &sba, const ros::Publisher &cam_pub, const ros::Publisher &pt_pub, int dec
          /*SysSPA &spa, ros::Publisher &cst_pub, ros::Publisher &link_pub*/)
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
  ptmark.scale.x = 0.02;
  ptmark.scale.y = 0.02;
  ptmark.scale.z = 0.02;
  ptmark.type = visualization_msgs::Marker::POINTS;

  cstmark = cammark;
  cstmark.color.r = 1.0f;
  cstmark.color.g = 1.0f;
  cstmark.color.b = 0.0f;
  cstmark.color.a = 1.0f;
  cstmark.scale.x = 0.03;
  cstmark.scale.y = 0.1;
  cstmark.scale.z = 0.1;
  cstmark.type = visualization_msgs::Marker::LINE_LIST;


  // draw points, decimated
  int npts = sba.tracks.size();

  cout << "Number of points to draw: " << npts << endl;
  if (npts <= 0) return;


  ptmark.points.resize(npts/dec+1);
  for (int i=0, ii=0; i<npts; i+=dec, ii++)
    {
      const Vector4d &pt = sba.tracks[i].point;
      ptmark.points[ii].x = pt(0);
      ptmark.points[ii].y = pt(2);
      ptmark.points[ii].z = -pt(1);
    }

  // draw cameras
  int ncams = sba.nodes.size();
  cammark.points.resize(ncams*6);
  for (int i=0, ii=0; i<ncams; i++)
    {
      const Node &nd = sba.nodes[i];
      Vector3d opt;
      Matrix<double,3,4> tr;
      transformF2W(tr,nd.trans,Quaternion<double>(nd.qrot));

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

#if 0
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
#endif // SPA

  cam_pub.publish(cammark);
  pt_pub.publish(ptmark);
  //cst_pub.publish(cstmark);
}



// main loop

int main(int argc, char** argv)
{
  if (argc < 8)
    {
      printf("Args are: <param file> <image dir> <left image file template> <right image file template> "
             "<vocab tree file> <vocab weights file> <calonder trees file>\n");
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


  // set up directories
  struct dirent **lims, **rims, **dirs;
  int nlim, nrim, ndirs;
  string dname = argv[2];
  if (!dname.compare(dname.size()-1,1,"/")) // see if slash at end
    dname.erase(dname.size()-1);

  string dirfn = dname.substr(dname.rfind("/")+1);
  string tdir = dname.substr(0,dname.rfind("/")+1);
  cout << "Top directory is " << tdir << endl;
  cout << "Search directory name is " << dirfn << endl;
  dreg = (char *)dirfn.c_str();

  ndirs = scandir(tdir.c_str(),&dirs,getidir,alphasort);
  printf("Found %d directories\n", ndirs);
  printf("%s\n",dirs[0]->d_name);

  // set up structures
  vslam::VslamSystem vslam(argv[5], argv[6]);
  typedef cv::CalonderDescriptorExtractor<float> Calonder;
  vslam.frame_processor_.setFrameDescriptor(new Calonder(argv[7]));
  
  // parameters
  vslam.setKeyDist(0.4);	// meters
  vslam.setKeyAngle(0.2);	// radians
  vslam.setKeyInliers(300);
  vslam.setHuber(2.0);          // Huber cost function cutoff
  //  vslam.vo_.pose_estimator_->wy = 64;
  //  vslam.vo_.pose_estimator_->wx = 64;
  //  vslam.vo_.pose_estimator_->numRansac = 1000;
  vslam.vo_.sba.verbose = false;
  vslam.sba_.verbose = false;
  
  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  ros::NodeHandle nh ("~");
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker>("points", 0);
  ros::Publisher cam_pub = nh.advertise<visualization_msgs::Marker>("cameras", 0);
  ros::Publisher cst_pub = nh.advertise<visualization_msgs::Marker>("constraints", 0);
  ros::Publisher link_pub = nh.advertise<visualization_msgs::Marker>("links", 0);

  // for RANSAC
  srand(mstime());

  // window
#ifdef VISMATCH  
  const std::string window_name = "VO tracks";
  cv::namedWindow(window_name,0);
  cv::Mat display;
#endif

  int iter = 0;
  lreg = argv[3];
  rreg = argv[4];


  // loop over directories
  for (int dd=0; dd<ndirs; dd++)
    {
      char dir[2048];
      sprintf(dir,"%s%s",tdir.c_str(),dirs[dd]->d_name);
      printf("Current directory: %s\n", dir);

      // get left/right image file names, sorted
      nlim = scandir(dir,&lims,getleft,alphasort);
      printf("Found %d left images\n", nlim);
      printf("%s\n",lims[0]->d_name);

      nrim = scandir(dir,&rims,getright,alphasort);
      printf("Found %d right images\n", nrim);
      printf("%s\n",rims[0]->d_name);

      if (nlim != nrim)
        {
          printf("Number of left/right images does not match: %d vs. %d\n", nlim, nrim);
          exit(0);
        }



      // loop over each stereo pair, adding it to the system
      for (int ii=0; ii<nlim; iter++, ii++)
        {
          // Load images
          char fn[2048];
          sprintf(fn,"%s/%s",dir,lims[ii]->d_name);
          cv::Mat image1 = cv::imread(fn,0);
          sprintf(fn,"%s/%s",dir,rims[ii]->d_name);
          cv::Mat image1r = cv::imread(fn,0);

#if 0
          printf("Image size: %d x %d\n", image1.cols, image1.rows);
          printf("Image size: %d x %d\n", image1r.cols, image1r.rows);
#endif

          if (image1.rows == 0 || image1r.rows == 0)
            exit(0);

          // Add stereo pair to the system
          bool is_keyframe = vslam.addFrame(camp, image1, image1r);

          if (is_keyframe)
            {
              /// @todo Depending on broken encapsulation of VslamSystem here
              int n = vslam.sba_.nodes.size();
              int np = vslam.sba_.tracks.size();

              // draw graph
              cout << "drawing with " << n << " nodes and " << np << " points..." << endl << endl;
#if 1
              if (n%2 == 0)
                drawgraph(vslam.sba_, cam_pub, pt_pub, 1); // every nth point
#endif

              // write file out
              if (n > 10 && n%500 == 0)
                {
                  char fn[1024];
                  sprintf(fn,"newcollege%d.g2o", n);
                  sba::writeGraphFile(fn,vslam.sba_);
                  sprintf(fn,"newcollege%dm.g2o", n);
                  sba::writeGraphFile(fn,vslam.sba_,true);
                  //                  sba::writeLourakisFile(fn, vslam.sba_);
                  //                  vslam.sba_.doSBA(1,1.0e-4,0);
                  //                  sba::writeSparseA(fn, vslam.sba_);
                }

#if 1
              int nnsba = 10;
              if (n > 4 && n%nnsba == 0)
                {
                  cout << "Running large SBA" << endl;
                  vslam.refine();
                }
#endif
            }

#ifdef VISMATCH
          // Visualize
          if (1)
            {
              drawVOtracks(image1, vslam.vo_.frames, display);
              cv::imshow(window_name, display);
              cv::waitKey(10);
            }
#endif

          if (!nh.ok())
            return 0;
        }
    }
  return 0;
}
