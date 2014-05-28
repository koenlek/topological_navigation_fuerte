#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/CvBridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/legacy/legacy.hpp>

#include <vslam_system/vslam.h>
#include <sba/visualization.h>
#include <vslam_system/any_detector.h>
#include <vslam_system/StereoVslamNodeConfig.h>

using namespace sba;
using namespace pcl;
using namespace Eigen;

void publishPointclouds(SysSBA& sba, ros::Publisher& pub);
void publishRegisteredPointclouds(sba::SysSBA& sba, 
    std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> >& frames, 
    ros::Publisher& pub);
void colorizePointcloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, cv::Mat& image);

class StereoVslamNode
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
        sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, 
        sensor_msgs::PointCloud2> SyncPolicy;

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;


  // Publications
  ros::Publisher cam_marker_pub_;
  ros::Publisher point_marker_pub_;
  image_transport::CameraPublisher vo_tracks_pub_;
  cv::Mat vo_display_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher registered_cloud_pub_ ;


  // Processing state
  sensor_msgs::CvBridge l_bridge_, r_bridge_;
  image_geometry::StereoCameraModel cam_model_;
  vslam::VslamSystem vslam_system_;
  cv::Ptr<cv::FeatureDetector> detector_;

  typedef dynamic_reconfigure::Server<vslam_system::StereoVslamNodeConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

public:

  StereoVslamNode(const std::string& vocab_tree_file, const std::string& vocab_weights_file,
                  const std::string& calonder_trees_file)
    : it_(nh_), sync_(3),
      vslam_system_(vocab_tree_file, vocab_weights_file),
      detector_(new vslam_system::AnyDetector)
  {
    // Use calonder descriptor
    typedef cv::CalonderDescriptorExtractor<float> Calonder;
    vslam_system_.frame_processor_.setFrameDescriptor(new Calonder(calonder_trees_file));

    
    // Advertise outputs
    cam_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/vslam/cameras", 1);
    point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/vslam/points", 1);
    vo_tracks_pub_ = it_.advertiseCamera("/vslam/vo_tracks/image", 1);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vslam/pointcloud", 1);
    registered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vslam/registered_pointcloud", 1);
  
    // Synchronize inputs
    l_image_sub_.subscribe(it_, "/narrow_stereo/left/image_rect", 10);
    l_info_sub_ .subscribe(nh_, "/narrow_stereo/left/camera_info", 10);
    r_image_sub_.subscribe(it_, "/narrow_stereo/right/image_rect", 10);
    r_info_sub_ .subscribe(nh_, "/narrow_stereo/right/camera_info", 10);
    point_sub_.subscribe(nh_, "/narrow_stereo_textured/points2", 10);
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_, point_sub_);
    sync_.registerCallback( boost::bind(&StereoVslamNode::imageCb, this, _1, _2, _3, _4, _5) );

    // Dynamic reconfigure for parameters
    ReconfigureServer::CallbackType f = boost::bind(&StereoVslamNode::configCb, this, _1, _2);
    reconfigure_server_.setCallback(f);

    vslam_system_.setPointcloudProc(boost::shared_ptr<frame_common::PointcloudProc>(new frame_common::PointcloudProc()));
    // Force vo keyframing parameters (fix this)
    vslam_system_.vo_.mindist = 0; ///< Minimum linear distance between keyframes (meters).
    vslam_system_.vo_.minang = 0;  ///< Minimum angular distance between keyframes (radians).
    vslam_system_.vo_.mininls = 0; ///< Minimum number of inliers.
    vslam_system_.setVOWindow(300,60);
    vslam_system_.setVORansacIt(2000);
    vslam_system_.setPRWindow(200,40);
    vslam_system_.nSkip = 1; 
    vslam_system_.setPlaceInliers(100);
    vslam_system_.setPRRansacIt(2000);
    vslam_system_.doPointPlane = false;
    vslam_system_.setHuber(2.0);
  }

  void configCb(vslam_system::StereoVslamNodeConfig& config, uint32_t level)
  {
    dynamic_cast<vslam_system::AnyDetector*>((cv::FeatureDetector*)detector_)->update(config);
    vslam_system_.frame_processor_.detector = detector_;
    
    vslam_system_.setPRRansacIt(config.pr_ransac_iterations);
    vslam_system_.setPRPolish(config.pr_polish);
    vslam_system_.setVORansacIt(config.vo_ransac_iterations);
    vslam_system_.setVOPolish(config.vo_polish);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
               const sensor_msgs::CameraInfoConstPtr& l_cam_info,
               const sensor_msgs::ImageConstPtr& r_image,
               const sensor_msgs::CameraInfoConstPtr& r_cam_info,
               const sensor_msgs::PointCloud2::ConstPtr& ptcloud_msg)
  {
    ROS_INFO("In callback, seq = %u", l_cam_info->header.seq);
    
    // Convert ROS messages for use with OpenCV
    cv::Mat left, right;
    try {
      left  = l_bridge_.imgMsgToCv(l_image, "mono8");
      right = r_bridge_.imgMsgToCv(r_image, "mono8");
    }
    catch (sensor_msgs::CvBridgeException& e) {
      ROS_ERROR("Conversion error: %s", e.what());
      return;
    }
    cam_model_.fromCameraInfo(l_cam_info, r_cam_info);

    frame_common::CamParams cam_params;
    cam_params.fx = cam_model_.left().fx();
    cam_params.fy = cam_model_.left().fy();
    cam_params.cx = cam_model_.left().cx();
    cam_params.cy = cam_model_.left().cy();
    cam_params.tx = cam_model_.baseline();
    
    pcl::PointCloud<pcl::PointXYZRGB> ptcloud;
    pcl::fromROSMsg(*ptcloud_msg, ptcloud);
    
    colorizePointcloud(ptcloud, left);

    if (vslam_system_.addFrame(cam_params, left, right, ptcloud)) 
    {
      int size = vslam_system_.sba_.nodes.size();

      if (vo_tracks_pub_.getNumSubscribers() > 0) {
        frame_common::drawVOtracks(left, vslam_system_.vo_.frames, vo_display_);
        IplImage ipl = vo_display_;
        sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
        msg->header = l_cam_info->header;
        vo_tracks_pub_.publish(msg, l_cam_info);
      }
      
      const int LARGE_SBA_INTERVAL = 1;
      if (size > 1 && size % LARGE_SBA_INTERVAL == 0)
      {
        ROS_INFO("Running large SBA on %d nodes", size);
        vslam_system_.refine(10);
      }
      
      /// @todo Not rely on broken encapsulation of VslamSystem here
      sba::drawGraph(vslam_system_.sba_, cam_marker_pub_, point_marker_pub_);

      if (pointcloud_pub_.getNumSubscribers() > 0)
        publishPointclouds(vslam_system_.sba_, pointcloud_pub_);
        
      if (registered_cloud_pub_.getNumSubscribers() > 0)
        publishRegisteredPointclouds(vslam_system_.sba_, vslam_system_.frames_, registered_cloud_pub_);
    }
  }
  
};

// TODO: inputs and outputs, but don't care for now.
void colorizePointcloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, cv::Mat& img)
{
    // Assign colors from the rectified image
  int i = 0;
  for (size_t u = 0; u < cloud.height; ++u)   // rows
  {
    for (size_t v = 0; v < cloud.width; ++v, ++i)  // cols
    {
      // If the point is invalid
      if (isnan (cloud.points[i].x) || isnan (cloud.points[i].y) || isnan (cloud.points[i].z))
        continue;
      // Get the color information
      uint8_t g = img.at<uint8_t>(u, v);
      int32_t rgb = (g << 16) | (g << 8) | g;
      cloud.points[i].rgb = *(float*)(&rgb);
    }
  }
}

void publishRegisteredPointclouds(sba::SysSBA& sba, 
    std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> >& frames, 
    ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> registered_cloud;

  for(size_t i=0; i < frames.size(); i++)
  {
    if (sba.nodes.size() < i)
      break;
    if (frames[i].dense_pointcloud.points.size() <= 0)
      continue;
    Eigen::Quaterniond rot = sba.nodes[i].qrot;
    Eigen::Vector3d trans = sba.nodes[i].trans.head<3>();
    
    transformPointCloud<PointXYZRGB>(frames[i].dense_pointcloud, cloud, Vector3f(0,0,0), rot.cast<float>());
    transformPointCloud<PointXYZRGB>(cloud, cloud, trans.cast<float>(), Quaternionf(1, 0, 0, 0));
    transformPointCloud<PointXYZRGB>(cloud, cloud, Vector3f(0,0,0), Quaternionf(.5, -.5, .5, -.5));
    
    registered_cloud.header = frames[i].dense_pointcloud.header;
    registered_cloud += cloud;
  }
  
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (registered_cloud, cloud_out);
  cloud_out.header.frame_id = "/pgraph";
  pub.publish (cloud_out);
}

void publishPointclouds(SysSBA& sba, ros::Publisher& pub)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.resize(sba.tracks.size());
  
    for(size_t i=0; i < sba.tracks.size(); i++)
    {
      ProjMap &prjs = sba.tracks[i].projections;
      //if (prjs.size() < 2) continue;
      
      bool pointplane = false;
      int lastframe = 0;
      
      for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
      {
        Proj &prj = itr->second;
        if (!prj.isValid) continue;
        if (prj.pointPlane)
          pointplane = true;
        lastframe = max(itr->first, lastframe);
      }
      
      cloud.points[i].x = sba.tracks[i].point(2);
      cloud.points[i].y = -sba.tracks[i].point(0);
      cloud.points[i].z = -sba.tracks[i].point(1);
      
      unsigned char r, g, b;
      
      if (!pointplane)
        //      { r = 255; g = 255; b = 255; }
      { r = 0; g = 0; b = 100; }
      else if (lastframe == 0)
      { r = 100; g = 100; b = 255; }
      else if (lastframe == 1)
      { r = 255; g = 0; b = 0; }
      else if (lastframe == 2)
      { r = 0; g = 255; b = 0; }
      else if (lastframe == 3)
      { r = 0; g = 0; b = 255; }
      else if (lastframe == 4)
      { r = 255; g = 0; b = 255; }
      else if (lastframe == 5)
      { r = 0; g = 255; b = 255; }
      else if (lastframe == 6)
      { r = 255; g = 255; b = 0; }
      else
      { r = 150; g = 150; b = 150; }
      
      int32_t rgb_packed = (r << 16) | (g << 8) | b;
      memcpy (&cloud.points[i].rgb, &rgb_packed, sizeof (int32_t));
    }
    
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg (cloud, cloud_out);
    cloud_out.header.frame_id = "/pgraph";
    pub.publish (cloud_out);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ptcloud_vslam");
  if (argc < 4) {
    printf("Usage: %s <vocab tree file> <vocab weights file> <calonder trees file>\n", argv[0]);
    return 1;
  }
  
  StereoVslamNode vslam(argv[1], argv[2], argv[3]);
  ros::spin();
}
