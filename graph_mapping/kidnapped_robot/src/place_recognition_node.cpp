#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <tf/transform_listener.h>
#include <kidnapped_robot/SavePlace.h>
#include <kidnapped_robot/MatchRequest.h>
#include <kidnapped_robot/MatchResult.h>
#include <opencv2/legacy/legacy.hpp> //KL: needs to be added to fix CalonderDescriptorExtractor build error

#include <cv_bridge/CvBridge.h>
#include <image_geometry/stereo_camera_model.h>

#include <kidnapped_robot/place_database.h>
#include <posest/pe3d.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <cmath>

template<class M>
boost::shared_ptr<M const> getCached(const ros::Time& time, const message_filters::Cache<M>& cache)
{
  boost::shared_ptr<M const> before = cache.getElemBeforeTime(time);
  return before;
}

class PlaceRecognitionNode
{
  // Subscriptions
  ros::Subscriber save_sub_, match_sub_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::Cache<sensor_msgs::Image> l_image_cache_, r_image_cache_;
  message_filters::Cache<sensor_msgs::CameraInfo> l_info_cache_, r_info_cache_;
  tf::TransformListener tf_;

  // Publications
  image_transport::Publisher image_sample_pub_, image_match_pub_;
  ros::Publisher transform_pub_;
  
  // Processing state
  kidnapped_robot::PlaceDatabase place_db_;
  frame_common::FrameProc frame_processor_;
  pe::PoseEstimator3d pose_estimator_;
  int pr_inliers_; // number of inliers needed for PR match

  // Message -> OpenCV conversions
  sensor_msgs::CvBridge l_bridge_, r_bridge_;
  image_geometry::StereoCameraModel cam_model_;
  
  std::string target_frame_;

public:
  PlaceRecognitionNode(const std::string& db_file,
                       const std::string& vocab_tree_file,
                       const std::string& vocab_weights_file,
                       const std::string& calonder_trees_file)
    : place_db_(db_file, vocab_tree_file, vocab_weights_file),
      frame_processor_(10),
      pose_estimator_(50000, false, 10.0, 3.0, 3.0),
      pr_inliers_(70)
  {
    ros::NodeHandle local_nh("~");
    int cache_size;
    local_nh.param("cache_size", cache_size, 10);
    l_image_cache_.setCacheSize(cache_size);
    l_info_cache_ .setCacheSize(cache_size);
    r_image_cache_.setCacheSize(cache_size);
    r_info_cache_ .setCacheSize(cache_size);
    
    // For matches, compute how this frame has moved
    local_nh.param<std::string>("target_frame", target_frame_, "/base_footprint");

    pose_estimator_.windowed = false;
    //pose_estimator_.wx = 92;
    //pose_estimator_.wy = 48;

    // Use calonder descriptor
    typedef cv::CalonderDescriptorExtractor<float> Calonder;
    frame_processor_.setFrameDescriptor(new Calonder(calonder_trees_file));
    
    // Subscriptions
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::NodeHandle stereo_nh("stereo"); // "stereo" namespace is remappable
    image_transport::ImageTransport stereo_it(stereo_nh);
    save_sub_ = nh.subscribe<kidnapped_robot::SavePlace>("save_image", 1, &PlaceRecognitionNode::saveCb, this);
    match_sub_ = nh.subscribe<kidnapped_robot::MatchRequest>("match", 1, &PlaceRecognitionNode::matchCb, this);
    // Cache all of the stereo topics
    l_image_sub_.subscribe(stereo_it, "left/image_rect", 1);
    l_image_cache_.connectInput(l_image_sub_);
    l_info_sub_ .subscribe(stereo_nh, "left/camera_info", 1);
    l_info_cache_.connectInput(l_info_sub_);
    r_image_sub_.subscribe(stereo_it, "right/image_rect", 1);
    r_image_cache_.connectInput(r_image_sub_);
    r_info_sub_ .subscribe(stereo_nh, "right/camera_info", 1);
    r_info_cache_.connectInput(r_info_sub_);

    // Publications
    image_sample_pub_ = it.advertise("image_samples", 1);
    image_match_pub_ = it.advertise("image_matches", 1);
    transform_pub_ = nh.advertise<kidnapped_robot::MatchResult>("transform_matches", 1);
  }

  void saveCb(const kidnapped_robot::SavePlaceConstPtr& msg)
  {
    // Pull messages for the requested time out of cache
    sensor_msgs::ImageConstPtr l_image = getCached(msg->stamp, l_image_cache_);
    sensor_msgs::ImageConstPtr r_image = getCached(msg->stamp, r_image_cache_);
    sensor_msgs::CameraInfoConstPtr l_info = getCached(msg->stamp, l_info_cache_);
    sensor_msgs::CameraInfoConstPtr r_info = getCached(msg->stamp, r_info_cache_);

    if (!l_image || !r_image || !l_info || !r_info)
    {
      ROS_WARN_STREAM("Unable to retrieve all messages at requested time " << msg->stamp << " : " << (bool) l_image
                      << ", " << (bool) r_image << ", " << (bool) l_info << ", " << (bool) r_info << "; range is "
                      << l_image_cache_.getOldestTime() << ", " << l_image_cache_.getLatestTime());
      return;
    }

    imageCb(l_image, l_info, r_image, r_info, true, msg->id);
  }

  void matchCb(const kidnapped_robot::MatchRequestConstPtr& msg)
  {
    // Pull messages for the requested time out of cache
    sensor_msgs::ImageConstPtr l_image = getCached(msg->stamp, l_image_cache_);
    sensor_msgs::ImageConstPtr r_image = getCached(msg->stamp, r_image_cache_);
    sensor_msgs::CameraInfoConstPtr l_info = getCached(msg->stamp, l_info_cache_);
    sensor_msgs::CameraInfoConstPtr r_info = getCached(msg->stamp, r_info_cache_);

    if (!l_image || !r_image || !l_info || !r_info)
    {
      ROS_WARN("Unable to retrieve all messages at requested time");
      return;
    }

    imageCb(l_image, l_info, r_image, r_info, false, 0);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
               const sensor_msgs::CameraInfoConstPtr& l_cam_info,
               const sensor_msgs::ImageConstPtr& r_image,
               const sensor_msgs::CameraInfoConstPtr& r_cam_info,
               bool save_in_db, uint32_t save_id)
  {
    /// @todo Should take into account that image and requested time stamps differ
    /// when doing tf transforms
    
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

    // Fill in Frame to be added
    frame_common::Frame frame;
    frame.setCamParams(cam_params);
    frame_processor_.setStereoFrame(frame, left, right);
    //frame.img = cv::Mat();
    frame.imgRight = cv::Mat(); // Don't bother keeping right image

    // Visualize keypoints
    if (image_sample_pub_.getNumSubscribers() > 0) {
      cv::Mat display;
      cv::drawKeypoints(left, frame.kpts, display, CV_RGB(255,0,0));
      IplImage display_ipl = display;
      image_sample_pub_.publish(l_bridge_.cvToImgMsg(&display_ipl, "bgr8"));
    }
    
    // Get head orientation from tf
    try {
      tf::StampedTransform camera_in_base;
      ros::Time cam_time = cam_model_.left().stamp();
      ros::Duration timeout(0.5);
      tf_.waitForTransform(target_frame_, cam_model_.tfFrame(), cam_time, timeout);
      tf_.lookupTransform(target_frame_, cam_model_.tfFrame(), cam_time, camera_in_base);

      if (save_in_db) {
        // Save the frame into the database, keyed by ID
        // For topological map, don't need the absolute pose in a global map
        int64_t id = place_db_.add(cam_time, tf::Pose(), camera_in_base, frame, save_id);
        ROS_INFO("Added place with id %li", id);
      }
      else {
        // Do place recognition
        const size_t N = 10; /// @todo Make parameterizable
        vt::Matches matches;
        place_db_.findMatching(frame, N, matches);

        printf("Matches (id, score, inliers):\n");
        for (int i = 0; i < (int)matches.size(); ++i) {
          int64_t id = matches[i].id;
          frame_common::Frame db_frame;
          place_db_.getFrame(id, db_frame);
          int inliers = pose_estimator_.estimate(db_frame, frame);
          printf("\t%li\t%f\t%i\n", id, matches[i].score, inliers);

          if (inliers > pr_inliers_) {
            tf::Pose db_base_in_map;
            tf::Transform db_camera_in_base;
            place_db_.getTransforms(id, db_base_in_map, db_camera_in_base);
            
            // Adjust using transform from pose estimator
            tf::Transform pe_transform;
            Eigen::Matrix3d rot = pose_estimator_.rot;
            pe_transform.getBasis().setValue(rot(0,0), rot(0,1), rot(0,2),
                                             rot(1,0), rot(1,1), rot(1,2),
                                             rot(2,0), rot(2,1), rot(2,2));
            Eigen::Vector3d trans = pose_estimator_.trans;
            pe_transform.getOrigin().setValue(trans[0], trans[1], trans[2]);
            /// @todo Rename to base_to_camera, etc.
            tf::Transform transform = db_camera_in_base * pe_transform * camera_in_base.inverse();

            publishTransform(transform, id);

            // Visualize matches
            if (image_match_pub_.getNumSubscribers() > 0) {
              cv::Mat display, left_db, right_db;
              place_db_.getImages(id, left_db, right_db);
              cv::drawMatches(left_db, db_frame.kpts, left, frame.kpts, pose_estimator_.inliers, display);
              IplImage display_ipl = display;
              image_match_pub_.publish(l_bridge_.cvToImgMsg(&display_ipl, "bgr8"));
            }
          }
        }
      }
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("TF exception:\n%s", ex.what());
      return;
    }
  }

  void publishTransform(const tf::Transform& transform, int64_t match_id)
  {
    kidnapped_robot::MatchResult result;
    tf::transformTFToMsg(transform, result.transform);
    result.match_id = match_id;
    transform_pub_.publish(result);
  }
};

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "place_recognition_node");

  if (argc < 5) {
    printf("Usage: %s places.db vocab.tree vocab.weights calonder.rtc\n", argv[0]);
    return 1;
  }

  PlaceRecognitionNode node(argv[1], argv[2], argv[3], argv[4]);

  ros::spin();
}
