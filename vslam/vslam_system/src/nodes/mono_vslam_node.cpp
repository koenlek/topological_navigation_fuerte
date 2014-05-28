#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/legacy/legacy.hpp>

#include <vslam_system/vslam_mono.h>
#include <sba/visualization.h>
#include <vslam_system/any_detector.h>
#include <vslam_system/StereoVslamNodeConfig.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

static const double PI = 3.14159265;

class StereoVslamNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  image_transport::SubscriberFilter l_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  // Publications
  ros::Publisher cam_marker_pub_;
  ros::Publisher point_marker_pub_;
  image_transport::CameraPublisher vo_tracks_pub_;
  cv::Mat vo_display_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcast_;
  tf::TransformListener tf_listener_;
  tf::Transform tf_transform_;

  // Processing state
  sensor_msgs::CvBridge l_bridge_, r_bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  vslam::VslamSystemMono vslam_system_;
  cv::Ptr<cv::FeatureDetector> detector_;

  typedef dynamic_reconfigure::Server<vslam_system::StereoVslamNodeConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

public:

  StereoVslamNode(const std::string& vocab_tree_file, const std::string& vocab_weights_file,
                  const std::string& calonder_trees_file)
    : it_(nh_), sync_(1),
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

    // Synchronize inputs
    l_image_sub_.subscribe(it_, "image_rect", 1);
    l_info_sub_ .subscribe(nh_, "camera_info", 1);
    sync_.connectInput(l_image_sub_, l_info_sub_);
    sync_.registerCallback( boost::bind(&StereoVslamNode::imageCb, this, _1, _2) );

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
               const sensor_msgs::CameraInfoConstPtr& l_cam_info)
  {
    ROS_INFO("In callback, seq = %u", l_cam_info->header.seq);
    
    // Convert ROS messages for use with OpenCV
    cv::Mat left;
    try 
    {
      left  = l_bridge_.imgMsgToCv(l_image, "mono8");
    }
    catch (sensor_msgs::CvBridgeException& e) 
    {
      ROS_ERROR("Conversion error: %s", e.what());
      return;
    }
    cam_model_.fromCameraInfo(l_cam_info);

    frame_common::CamParams cam_params;
    cam_params.fx = cam_model_.fx();
    cam_params.fy = cam_model_.fy();
    cam_params.cx = cam_model_.cx();
    cam_params.cy = cam_model_.cy();
    cam_params.tx = 0.0;

    if (vslam_system_.addFrame(cam_params, left)) {
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
      
      // Publish odometry data to tf.
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
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_vslam");
  if (argc < 4) {
    printf("Usage: %s <vocab tree file> <vocab weights file> <calonder trees file>\n", argv[0]);
    return 1;
  }
  
  StereoVslamNode vslam(argv[1], argv[2], argv[3]);
  ros::spin();
}
