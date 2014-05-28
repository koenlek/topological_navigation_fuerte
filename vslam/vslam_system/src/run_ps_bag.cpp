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
// runs a sequence of depth/visual images from a PS sensor into an SBA engine
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
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>
#include <time.h>
#include <opencv/highgui.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

using namespace std;
using namespace sba;
using namespace frame_common;
using namespace Eigen;
using namespace vslam;
using namespace pcl;

// visual output of matches
#define VISMATCH


class VisBundler
{
  public:
    VisBundler( const string& bag_filename, const string& vocabulary_tree_filename,
                const string& voc_weights_filename, cv::DescriptorExtractor* descriptor_extractor );
    ~VisBundler();

    bool step();
    void spin();
  protected:
    void publishRegisteredPointclouds( ) const;
    void drawgraph( int dec ) const;

    // member vaiables
    ros::NodeHandle node_handle_;
    vslam::VslamSystem vslam_;
    rosbag::Bag bag_;
    rosbag::View view_;
    rosbag::View::iterator message_iterator_;
    ros::Publisher pt_pub_;
    ros::Publisher cam_pub_;
    ros::Publisher pc_pub_;
};

VisBundler::VisBundler( const string& bag_filename, const string& vocabulary_tree_filename,
                        const string& vocabulary_weights_filename, cv::DescriptorExtractor* descriptor_extractor )
        : node_handle_("~")
        , vslam_( vocabulary_tree_filename, vocabulary_weights_filename )
        , bag_( bag_filename )
        , view_( bag_ )
        , message_iterator_( view_.begin() )
        , pt_pub_( node_handle_.advertise<visualization_msgs::Marker>("points", 0) )
        , cam_pub_( node_handle_.advertise<visualization_msgs::Marker>("cameras", 0) )
        , pc_pub_( node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1) )
{
  vslam_.frame_processor_.setFrameDescriptor( descriptor_extractor );
  vslam_.setPointcloudProc(boost::shared_ptr<frame_common::PointcloudProc>(new frame_common::PointcloudProc()));
  vslam_.doPointPlane = false;
  
  // parameters
  vslam_.setKeyDist(0.01);	// meters
  vslam_.setKeyAngle(0.05);	// radians
  vslam_.setKeyInliers(200);
  vslam_.setHuber(40.0);          // Huber cost function cutoff
  vslam_.vo_.pose_estimator_->wy = 128;
  vslam_.vo_.pose_estimator_->wx = 128;
  vslam_.vo_.pose_estimator_->numRansac = 10000;
  vslam_.vo_.sba.verbose = false;
  vslam_.sba_.verbose = false;
  
  srand(time(0));

  #ifdef VISMATCH
  cv::namedWindow("VO tracks",0);
  cv::namedWindow("color image");
  #endif
}

VisBundler::~VisBundler()
{
}

void VisBundler::publishRegisteredPointclouds( ) const
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> registered_cloud;

  for(size_t i=0; i < vslam_.vo_.frames.size(); i++)
  {
    //    printf("Ptcloud size: %d\n", frames[i].dense_pointcloud.points.size());
    if (vslam_.vo_.sba.nodes.size() < i)
      break;
    if (vslam_.vo_.frames[i].dense_pointcloud.points.size() <= 0)
      continue;
    Eigen::Quaterniond rot = vslam_.vo_.sba.nodes[i].qrot;
    Eigen::Vector3d trans = vslam_.vo_.sba.nodes[i].trans.head<3>();

    transformPointCloud<PointXYZRGB>(vslam_.vo_.frames[i].dense_pointcloud, cloud, Vector3f(0,0,0), rot.cast<float>());
    transformPointCloud<PointXYZRGB>(cloud, cloud, trans.cast<float>(), Quaternionf(1, 0, 0, 0));
    // rotate into rviz frame from camera frame
    Quaternionf qr = Quaternionf(.5, 0, 0, .5).normalized()*Quaternionf(.5, -.5, .5, -0.5).normalized();
    transformPointCloud<PointXYZRGB>(cloud, cloud, Vector3f(0,0,0), qr);

    registered_cloud.header = vslam_.vo_.frames[i].dense_pointcloud.header;
    registered_cloud += cloud;
  }

  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (registered_cloud, cloud_out);
  cloud_out.header.frame_id = "/pgraph";
  pc_pub_.publish (cloud_out);
}


// draw the graph on rviz
// in rviz frame, z -> y, x -> -z from camera frame
void VisBundler::drawgraph( int dec ) const
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
  ptmark.scale.x = 0.01;
  ptmark.scale.y = 0.01;
  ptmark.scale.z = 0.01;
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
  int npts = vslam_.vo_.sba.tracks.size();

  //  cout << "Number of points to draw: " << npts << endl;
  if (npts <= 0) return;


  ptmark.points.resize(npts/dec+1);
  for (int i=0, ii=0; i<npts; i+=dec, ii++)
    {
      const Vector4d &pt = vslam_.vo_.sba.tracks[i].point;
      ptmark.points[ii].x = pt(0);
      ptmark.points[ii].y = pt(2);
      ptmark.points[ii].z = -pt(1);
    }

  // draw cameras
  int ncams = vslam_.vo_.sba.nodes.size();
  cammark.points.resize(ncams*6);
  for (int i=0, ii=0; i<ncams; i++)
    {
      const Node &nd = vslam_.vo_.sba.nodes[i];
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

  // draw point-plane projections
  int num_tracks = vslam_.vo_.sba.tracks.size();
  int ii = cammark.points.size();

  for (int i=0; i < num_tracks; i++)
    {
      const ProjMap &prjs = vslam_.vo_.sba.tracks[i].projections;
      for (ProjMap::const_iterator itr = prjs.begin(); itr != prjs.end(); itr++)
        {
          const Proj &prj = (*itr).second;
          if (prj.pointPlane)	// have a ptp projection
            {
              cammark.points.resize(ii+2);
              Point pt0 = vslam_.vo_.sba.tracks[i].point;
              Vector3d plane_point = prj.plane_point;
              Vector3d plane_normal = prj.plane_normal;
              Eigen::Vector3d w = pt0.head<3>()-plane_point;
              //              Eigen::Vector3d projpt = plane_point+(w.dot(plane_normal))*plane_normal;
              Eigen::Vector3d projpt = pt0.head<3>() - (w.dot(plane_normal))*plane_normal;
              //              Vector3d pt1 = pt0.head<3>()+0.1*plane_normal;
              Vector3d pt1 = projpt;
	          
              cammark.points[ii].x = pt0.x();
              cammark.points[ii].y = pt0.z();
              cammark.points[ii++].z = -pt0.y();
              cammark.points[ii].x = pt1.x();
              cammark.points[ii].y = pt1.z();
              cammark.points[ii++].z = -pt1.y();
            }
        } 
    }


#if 0
  // draw SPA constraints
  int ncons = vslam_.vo_.spa.p2cons.size();
  cstmark.points.resize(ncons*6);

  for (int i=0, ii=0; i<ncons; i++)
    {
      ConP2 &con = vslam_.vo_.spa.p2cons[i];
      Node &nd0 = vslam_.vo_.spa.nodes[con.ndr];
      Node &nd1 = vslam_.vo_.spa.nodes[con.nd1];

      Node &nd = vslam_.vo_.spa.nodes[i];
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

  cam_pub_.publish(cammark);
  pt_pub_.publish(ptmark);
  
  //cst_pub.publish(cstmark);
}


bool VisBundler::step( )
{
  static boost::shared_ptr< sensor_msgs::CameraInfo > camera_info_left;
  static boost::shared_ptr< sensor_msgs::CameraInfo > camera_info_right;
  static boost::shared_ptr< sensor_msgs::Image > color_image;
  static boost::shared_ptr< stereo_msgs::DisparityImage > disparity_image;

  static int message_count = 0;
  while( message_iterator_ != view_.end() )
  {
    if( message_iterator_->getTopic() == "/primesense/left/camera_info" )
    {
      camera_info_left = message_iterator_->instantiate<sensor_msgs::CameraInfo>();
    }

    else if( message_iterator_->getTopic() == "/primesense/right/camera_info" )
    {
      camera_info_right = message_iterator_->instantiate<sensor_msgs::CameraInfo>();
    }

    else if( message_iterator_->getTopic() == "/primesense/left/color_image_raw" )
    {
      color_image = message_iterator_->instantiate<sensor_msgs::Image>();
    }
    else if( message_iterator_->getTopic() == "/primesense/disparity" )
    {
      disparity_image = message_iterator_->instantiate<stereo_msgs::DisparityImage>();
    }
    
    ++message_iterator_;
    ++message_count;

    if( message_count > 3 )
    {
      ros::Time oldest = min( min( color_image->header.stamp, camera_info_right->header.stamp ) ,
                              min( camera_info_left->header.stamp, disparity_image->header.stamp ) );

      ros::Time newest = max( max( color_image->header.stamp, camera_info_right->header.stamp ) ,
                              max( camera_info_left->header.stamp, disparity_image->header.stamp ) );

      if( oldest == newest )
      {
        CamParams camera_params;
        camera_params.fx = camera_info_left->K[0];
        camera_params.fy = camera_info_left->K[4];
        camera_params.cx = camera_info_left->K[2];
        camera_params.cx = camera_info_left->K[5];
        camera_params.tx = - camera_info_right->P[3] / camera_params.fx;

        static sensor_msgs::CvBridge cvImageBridge;
        cv::Mat image( cvImageBridge.imgMsgToCv( color_image ) );

        cv::Mat grayImage;
        cv::cvtColor( image, grayImage, CV_RGB2GRAY );
        static sensor_msgs::CvBridge cvDisparityBridge;
        cvDisparityBridge.fromImage( disparity_image->image );
        cv::Mat disp_image32f = cvDisparityBridge.toIpl();
        cv::Mat disp_image16u;

        disp_image32f.convertTo( disp_image16u, CV_16UC1, 32.0 );

        bool is_keyframe = vslam_.addFrame( camera_params, grayImage, disp_image16u, 32.0, true );

        if (is_keyframe)
        {

          imshow("color image", image );
          vslam_.sba_.doSBA(10,1e-4,SBA_SPARSE_CHOLESKY);

          /// @todo Depending on broken encapsulation of VslamSystem here
          int n = vslam_.sba_.nodes.size();

          // draw graph
          //              cout << "drawing with " << n << " nodes and " << np << " points..." << endl << endl;
          if (n%1 == 0)
          {
            drawgraph( 1 ); // every nth point
            publishRegisteredPointclouds( );
            drawgraph( 1 ); // every nth point
            publishRegisteredPointclouds( );
            for (int i=0; i<5; i++)
              {
                //                      getchar();
                cout << i << endl;
                vslam_.vo_.sba.doSBA( 1,1.0e-5,0 );          // dense version
                drawgraph( 1 ); // every nth point
                publishRegisteredPointclouds( );
                drawgraph( 1 ); // every nth point
                publishRegisteredPointclouds( );
              }
          }

          // write file out
          if (n > 10 && n%500 == 0)
          {
            char fn[1024];
            sprintf(fn,"newcollege%d.g2o", n);
            sba::writeGraphFile(fn,vslam_.sba_);
            sprintf(fn,"newcollege%dm.g2o", n);
            sba::writeGraphFile(fn,vslam_.sba_,true);
          }

          int nnsba = 10;
          if (n > 4 && n%nnsba == 0)
          {
            cout << "Running large SBA" << endl;
            vslam_.refine();
          }

          break;
        }
#ifdef VISMATCH
        cv::Mat display;
        drawVOtracks(grayImage, vslam_.vo_.frames, display);
        cv::imshow("VO tracks", display);
#endif
      }
    }
  }

  return (message_iterator_ != view_.end());
}
// main loop

void VisBundler::spin()
{
  bool quit = false;
  bool paused = true;
  while( node_handle_.ok() && !quit )
  {
    ros::spinOnce();
    char key = cv::waitKey( 10 ) & 0xFF;
    switch( key )
    {
      case 27:
      case 'Q':
      case 'q':
        quit = true;
        break;
      case ' ':
        paused = !paused;
        break;
      case 's':
      case 'S':
        paused = true;
        step();
        break;
    }
    if( !paused )
    {
      step();
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    cout <<"Usage: " << argv[0] << " <bag file> <vocab tree file> <vocab weights file> <calonder trees file>" << endl;
    exit(0);
  }
  
  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  typedef cv::CalonderDescriptorExtractor<float> Calonder;
  typedef cv::SiftDescriptorExtractor Sift;
  VisBundler visbundler( argv[1], argv[2], argv[3], new Calonder(argv[4]) );

  visbundler.spin();

  return 0;
}
