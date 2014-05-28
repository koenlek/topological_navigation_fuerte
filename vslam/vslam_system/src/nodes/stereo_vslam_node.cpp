#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/CvBridge.h>
#include <image_geometry/stereo_camera_model.h>

#include <vslam_system/vslam.h>
#include <sba/visualization.h>
#include <vslam_system/any_detector.h>
#include <vslam_system/StereoVslamNodeConfig.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/legacy/legacy.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void publishRegisteredPointclouds(sba::SysSBA& sba, 
    std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> >& frames, 
    ros::Publisher& pub);

class StereoVslamNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  // Publications
  ros::Publisher cam_marker_pub_;
  ros::Publisher point_marker_pub_;
  image_transport::CameraPublisher vo_tracks_pub_;
  cv::Mat vo_display_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcast_;
  tf::TransformListener tf_listener_;
  tf::Transform tf_transform_;
  ros::Publisher pointcloud_pub_;

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
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/vo", 1);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vslam/pointcloud", 1);

    // Synchronize inputs
    l_image_sub_.subscribe(it_, "left/image_rect", 1);
    l_info_sub_ .subscribe(nh_, "left/camera_info", 1);
    r_image_sub_.subscribe(it_, "right/image_rect", 1);
    r_info_sub_ .subscribe(nh_, "right/camera_info", 1);
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&StereoVslamNode::imageCb, this, _1, _2, _3, _4) );

    // Dynamic reconfigure for parameters
    ReconfigureServer::CallbackType f = boost::bind(&StereoVslamNode::configCb, this, _1, _2);
    reconfigure_server_.setCallback(f);

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
               const sensor_msgs::CameraInfoConstPtr& r_cam_info)
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

    if (vslam_system_.addFrame(cam_params, left, right)) {
      /// @todo Not rely on broken encapsulation of VslamSystem here
      int size = vslam_system_.sba_.nodes.size();
      if (size % 2 == 0) {
        // publish markers
        sba::drawGraph(vslam_system_.sba_, cam_marker_pub_, point_marker_pub_);
      }

      // Publish VO tracks
      if (vo_tracks_pub_.getNumSubscribers() > 0) {
        frame_common::drawVOtracks(left, vslam_system_.vo_.frames, vo_display_);
        IplImage ipl = vo_display_;
        sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
        msg->header = l_cam_info->header;
        vo_tracks_pub_.publish(msg, l_cam_info);
      }
      
      // Refine large-scale SBA.
      const int LARGE_SBA_INTERVAL = 10;
      if (size > 4 && size % LARGE_SBA_INTERVAL == 0) {
        ROS_INFO("Running large SBA on %d nodes", size);
        vslam_system_.refine();
      }
      
      if (pointcloud_pub_.getNumSubscribers() > 0 && size % 2 == 0)
        publishRegisteredPointclouds(vslam_system_.sba_, vslam_system_.frames_, pointcloud_pub_);
      
      // Publish odometry data to tf.
      if (0) // TODO: Change this to parameter.
      {
        ros::Time stamp = l_cam_info->header.stamp;
        std::string image_frame = l_cam_info->header.frame_id;
        Eigen::Vector4d trans = -vslam_system_.sba_.nodes.back().trans;
        Eigen::Quaterniond rot = vslam_system_.sba_.nodes.back().qrot.conjugate();
        
        trans.head<3>() = rot.toRotationMatrix()*trans.head<3>(); 
        
        tf_transform_.setOrigin(tf::Vector3(trans(0), trans(1), trans(2)));
        tf_transform_.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()) );
        
        tf::Transform simple_transform;
        simple_transform.setOrigin(tf::Vector3(0, 0, 0));
        simple_transform.setRotation(tf::Quaternion(.5, -.5, .5, .5));
        
        tf_broadcast_.sendTransform(tf::StampedTransform(tf_transform_, stamp, image_frame, "visual_odom"));
        tf_broadcast_.sendTransform(tf::StampedTransform(simple_transform, stamp, "visual_odom", "pgraph"));
      
      
        // Publish odometry data on topic.
        if (odom_pub_.getNumSubscribers() > 0)
        {
          tf::StampedTransform base_to_image;
          tf::Transform base_to_visodom;
         
          try
          {
            tf_listener_.lookupTransform(image_frame, "/base_footprint",
                                 stamp, base_to_image);
          }
          catch (tf::TransformException ex)
          {
              ROS_WARN("%s",ex.what());
              return;
          }
                                 
          base_to_visodom = tf_transform_.inverse() * base_to_image;
          
          geometry_msgs::PoseStamped pose;
          nav_msgs::Odometry odom;
          pose.header.frame_id = "/visual_odom";
          pose.pose.position.x = base_to_visodom.getOrigin().x();
          pose.pose.position.y = base_to_visodom.getOrigin().y();
          pose.pose.position.z = base_to_visodom.getOrigin().z();
          pose.pose.orientation.x = base_to_visodom.getRotation().x();
          pose.pose.orientation.y = base_to_visodom.getRotation().y();
          pose.pose.orientation.z = base_to_visodom.getRotation().z();
          pose.pose.orientation.w = base_to_visodom.getRotation().w();
          
          odom.header.stamp = stamp;
          odom.header.frame_id = "/visual_odom";
          odom.child_frame_id = "/base_footprint";
          odom.pose.pose = pose.pose;
          /* odom.pose.covariance[0] = 1;
          odom.pose.covariance[7] = 1;
          odom.pose.covariance[14] = 1;
          odom.pose.covariance[21] = 1;
          odom.pose.covariance[28] = 1;
          odom.pose.covariance[35] = 1; */
          odom_pub_.publish(odom);
        }
      }
    }
  }
};

void publishRegisteredPointclouds(sba::SysSBA& sba, 
    std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> >& frames, 
    ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  
  // Count the number of points in the system:
  unsigned int totalpoints = 0;
  for (size_t i = 0; i < frames.size(); i++)
  {
    totalpoints += frames[i].pts.size();
  }
  
  cloud.points.resize(totalpoints);
  unsigned int k = 0;

  for(size_t i=0; i < frames.size(); i++)
  {
    if (sba.nodes.size() < i)
      break;
    Eigen::Matrix3d rotmat = sba.nodes[i].qrot.toRotationMatrix();
    Eigen::Vector3d trans = sba.nodes[i].trans.head<3>();
    
    for (size_t j=0; j < frames[i].pts.size(); j++)
    {
      // If too close or too far away, just ignore.
      if (frames[i].pts[j].z() > 30.0 || frames[i].pts[j].z() < 0.1)
        continue;
      Eigen::Vector3d point(frames[i].pts[j].x(), frames[i].pts[j].y(), frames[i].pts[j].z());
      point = rotmat*point + trans;
      
      cloud.points[k].x = point(2);
      cloud.points[k].y = -point(0);
      cloud.points[k].z = -point(1);
    
      unsigned char r(255), g(255), b(255);
      
      int32_t rgb_packed = (r << 16) | (g << 8) | b;
      memcpy (&cloud.points[k].rgb, &rgb_packed, sizeof (int32_t));
      k++;
    }
  }
  
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (cloud, cloud_out);
  cloud_out.header.frame_id = "/pgraph";
  pub.publish (cloud_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_vslam");
  if (argc < 4) {
    printf("Usage: %s <vocab tree file> <vocab weights file> <calonder trees file>\n", argv[0]);
    return 1;
  }
  
  StereoVslamNode vslam(argv[1], argv[2], argv[3]);
  ros::spin();
}
