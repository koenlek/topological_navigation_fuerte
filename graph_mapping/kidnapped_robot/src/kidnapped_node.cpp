#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <topic_tools/MuxSelect.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <kidnapped_robot/RecognizePlace.h>

#include <cv_bridge/CvBridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/legacy/legacy.hpp> //KL: needs to be added to fix CalonderDescriptorExtractor build error

#include <kidnapped_robot/place_database.h>
#include <posest/pe3d.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <cmath>

using pr2_controllers_msgs::PointHeadAction;
typedef actionlib::SimpleActionClient<PointHeadAction> PointHeadClient;

class KidnappedNode
{
  ros::NodeHandle nh_;
  ros::CallbackQueue stereo_cb_queue_;
  ros::NodeHandle stereo_nh_; // In stereo namespace
  boost::scoped_ptr<image_transport::ImageTransport> it_;
  boost::scoped_ptr<ros::AsyncSpinner> spinner_;

  // Subscriptions
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  ros::Subscriber localization_sub_; // poses from AMCL
  tf::TransformListener tf_;

  // Publications
  image_transport::Publisher image_sample_pub_, image_match_pub_;
  geometry_msgs::PoseArray sample_poses_;
  ros::Publisher sample_pose_pub_;
  ros::Publisher initial_pose_pub_; // initialize AMCL
  ros::Publisher initial_pose_viz_pub_; // just the pose, for visualization
  
  // Services
  ros::ServiceClient mux_client_; // for halting the base when collecting data
  ros::ServiceServer pr_server_; // localize using place recognition
  
  // Processing state
  kidnapped_robot::PlaceDatabase place_db_;
  frame_common::FrameProc frame_processor_;
  pe::PoseEstimator3d pose_estimator_;
  int pr_inliers_; // number of inliers needed for PR match
  int num_samples_; // how many images to collect at each place
  double distance_threshold_; // size of bounding box when deciding whether to take new sample
  bool collect_new_data_; // whether to add to database or do PR

  // Synchronization primitives and shared data
  /// @todo Probably could have kept this single-threaded, just do own spin on stereo_cb_queue_
  boost::condition_variable sample_cond_;
  boost::mutex sample_mutex_;
  bool take_sample_;
  tf::Stamped<tf::Pose> best_pose_;
  int best_inliers_;

  // Message -> OpenCV conversions
  sensor_msgs::CvBridge l_bridge_, r_bridge_;
  image_geometry::StereoCameraModel cam_model_;
  
  PointHeadClient point_head_client_;
  std::string camera_optical_frame_, target_frame_, map_frame_;
  double head_target_z_;

public:
  KidnappedNode(const std::string& db_file, const std::string& vocab_tree_file,
                const std::string& vocab_weights_file, const std::string& calonder_trees_file)
    : stereo_nh_("stereo"), sync_(3),
      place_db_(db_file, vocab_tree_file, vocab_weights_file),
      frame_processor_(10),
      pose_estimator_(50000, true, 10.0, 3.0, 3.0),
      pr_inliers_(70),
      point_head_client_("/head_traj_controller/point_head_action", true)
  {
    // Image processing will be done on separate callback queue so we can manage it within poseCb
    stereo_nh_.setCallbackQueue(&stereo_cb_queue_);
    it_.reset(new image_transport::ImageTransport(stereo_nh_));
    spinner_.reset(new ros::AsyncSpinner(1, &stereo_cb_queue_));
    
    camera_optical_frame_ = "/wide_stereo_optical_frame";
    target_frame_ = "/base_footprint";
    map_frame_ = "/map";
    head_target_z_ = 1.2;
    num_samples_ = 5;
    distance_threshold_ = 1.0;

    pose_estimator_.windowed = false;
    //pose_estimator_.wx = 92;
    //pose_estimator_.wy = 48;

    // Use calonder descriptor
    typedef cv::CalonderDescriptorExtractor<float> Calonder;
    frame_processor_.setFrameDescriptor(new Calonder(calonder_trees_file));
    
    // Wait for head controller action server to come up
    while (ros::ok() && !point_head_client_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }

    // Services
    ros::NodeHandle mux_nh("mux");
    mux_client_ = mux_nh.serviceClient<topic_tools::MuxSelect>("select");

    pr_server_ = nh_.advertiseService("recognize_place", &KidnappedNode::localizeCb, this);

    // Subscriptions
    localization_sub_ = nh_.subscribe("map_pose", 0, &KidnappedNode::poseCb, this);
    // Synchronize stereo inputs, but don't subscribe yet
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&KidnappedNode::imageCb, this, _1, _2, _3, _4) );

    // Publications
    image_sample_pub_ = it_->advertise("/image_samples", 1);
    image_match_pub_ = it_->advertise("/pr_matches", 1);
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    initial_pose_viz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("place_localized_pose", 1);
    
    sample_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("place_poses", 1, true);
    loadDbSamplePoses();
    publishSamplePoses();
  }

  void loadDbSamplePoses()
  {
    sqlite3 *db = place_db_.getSqlite();
    static const char SELECT_IDS[] = "SELECT id FROM places";
    sqlite3_stmt *stmt = NULL;
    assert(sqlite3_prepare_v2(db, SELECT_IDS, sizeof(SELECT_IDS), &stmt, NULL) == SQLITE_OK);

    tf::Pose base_in_map;
    tf::Transform camera_in_base;
    int err;
    while ((err = sqlite3_step(stmt)) == SQLITE_ROW) {
      int64_t id = sqlite3_column_int64(stmt, 0);
      place_db_.getTransforms(id, base_in_map, camera_in_base);
      addToSamplePoses(base_in_map, camera_in_base);
    }
    assert(err == SQLITE_DONE);

    err = sqlite3_finalize(stmt);
    assert(err == SQLITE_OK);
  }

  void publishSamplePoses()
  {
    sample_poses_.header.stamp = ros::Time::now();
    sample_poses_.header.frame_id = map_frame_;
    sample_pose_pub_.publish(sample_poses_);
  }

  void subscribeStereo()
  {
    l_image_sub_.subscribe(*it_, "left/image_rect", 1);
    l_info_sub_ .subscribe(stereo_nh_, "left/camera_info", 1);
    r_image_sub_.subscribe(*it_, "right/image_rect", 1);
    r_info_sub_ .subscribe(stereo_nh_, "right/camera_info", 1);
  }

  void unsubscribeStereo()
  {
    l_image_sub_.unsubscribe();
    l_info_sub_ .unsubscribe();
    r_image_sub_.unsubscribe();
    r_info_sub_ .unsubscribe();
  }

  //! Points the camera frame at a point in the target frame
  void lookAt(double x, double y, double z)
  {
    // The goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    // The target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = target_frame_;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    // Point the camera optical frame, which is Z-forward
    goal.pointing_frame = camera_optical_frame_;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;

    // Go no faster than 1 rad/s
    //goal.max_velocity = 1.0;

    //send the goal
    point_head_client_.sendGoal(goal);

    //wait for it to get there
    point_head_client_.waitForResult();

    if (point_head_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_WARN("lookAt failed!");
  }

  void poseCb(const geometry_msgs::PoseWithCovarianceStamped& pose_estimate)
  {
    ROS_DEBUG("Got a map pose estimate");

    // Reject if covariance is too high
    const double MAX_VARIANCE = 0.05;
    double xx = pose_estimate.pose.covariance[0], yy = pose_estimate.pose.covariance[7];
    if (xx > MAX_VARIANCE || yy > MAX_VARIANCE) {
      ROS_INFO("Reject map pose estimate due to high variance: xx = %f, yy = %f", xx, yy);
      return;
    }

    // Check if already close to known places in the DB
    cv::Rect_<double> bounding_box;
    bounding_box.x = pose_estimate.pose.pose.position.x - distance_threshold_ / 2.0;
    bounding_box.y = pose_estimate.pose.pose.position.y - distance_threshold_ / 2.0;
    bounding_box.width = bounding_box.height = distance_threshold_;
    std::vector<int64_t> ids;
    place_db_.findInRegion(bounding_box, ids);
    if (ids.size() > 0) {
      ROS_INFO("Ignoring pose estimate, already have %d records nearby", (int)ids.size());
#if 0
      printf("\tids: ");
      for (int i = 0; i < ids.size(); ++i)
        printf("%li ", ids[i]);
      printf("\n");
#endif
      return;
    }

    collect_new_data_ = true;
    collectImages(num_samples_);
  }

  //! Collect n images evenly spaced around the robot
  void collectImages(int n)
  {
    // Halt the robot
    topic_tools::MuxSelect select_srv;
    select_srv.request.topic = "__none"; // don't forward any velocity commands
    if (!mux_client_.call(select_srv)) {
      ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly?",
                mux_client_.getService().c_str());
      return;
    }

    // Start receiving stereo images
    take_sample_ = false;
    subscribeStereo();
    spinner_->start();
    
    const double BLIND_ANGLE = M_PI / 8.0; // PR2 can't look straight back

    // Collect n images spanning robot's surroundings
    double step = 2.0 * (M_PI - BLIND_ANGLE) / (n - 1);
    double angle = -(M_PI - BLIND_ANGLE);
    for (int i = 0; i < n; ++i) {
      double x = 3.0 * std::cos(angle);
      double y = 3.0 * std::sin(angle);
      ROS_DEBUG("Looking at (%f, %f, %f)", x, y, head_target_z_);
      lookAt(x, y, head_target_z_);

      angle += step;
      ros::Duration(1.0).sleep(); // Wait for head to settle

      boost::unique_lock<boost::mutex> lock(sample_mutex_);
      take_sample_ = true;
      while (take_sample_) {
        sample_cond_.wait(lock);
      }
    }

    spinner_->stop();

    // Point head forward before moving again
    lookAt(3.0, 0.0, head_target_z_);

    // Restore move_base mux to previous input topic
    select_srv.request.topic = select_srv.response.prev_topic;
    if (!mux_client_.call(select_srv))
      ROS_WARN("Failed to reset mux to previous topic");

    publishSamplePoses();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
               const sensor_msgs::CameraInfoConstPtr& l_cam_info,
               const sensor_msgs::ImageConstPtr& r_image,
               const sensor_msgs::CameraInfoConstPtr& r_cam_info)
  {
    if (!take_sample_) {
      ROS_DEBUG("Not taking sample, seq = %u", l_cam_info->header.seq);
      return;
    }
    ROS_DEBUG("Taking sample, seq = %u", l_cam_info->header.seq);
    
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

      if (collect_new_data_) {
        /// @todo Don't have to look up base_in_map every time, but arguably more robust this way
        tf::StampedTransform base_in_map;
        tf_.waitForTransform(map_frame_, target_frame_, cam_time, timeout);
        tf_.lookupTransform(map_frame_, target_frame_, cam_time, base_in_map);
      
        // Add new place record to the DB
        int64_t id = place_db_.add(cam_time, base_in_map, camera_in_base, frame);
        ROS_INFO("Added place %li with base in map (x,y) = (%f, %f)", id,
                 base_in_map.getOrigin().x(), base_in_map.getOrigin().y());

        addToSamplePoses(base_in_map, camera_in_base); // for visualization
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

          if (inliers > best_inliers_ && inliers > pr_inliers_) {
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
            tf::Pose map_pose = db_base_in_map * db_camera_in_base * pe_transform * camera_in_base.inverse();
            best_pose_.setData(map_pose);
            
            best_pose_.frame_id_ = map_frame_;
            best_pose_.stamp_ = cam_time;
            best_inliers_ = inliers;

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
    
    {
      boost::lock_guard<boost::mutex> lock(sample_mutex_);
      take_sample_ = false;
    }
    sample_cond_.notify_one();
  }

  void addToSamplePoses(const tf::Transform& base_in_map, const tf::Transform& camera_in_base)
  {
    tf::Transform camera_in_map = base_in_map * camera_in_base;
    // Came out 90 degrees off in rviz for some reason, correct it here...
    double yaw, pitch, roll;
    camera_in_map.getBasis().getEulerYPR(yaw, pitch, roll);
    yaw += M_PI/2;
    camera_in_map.getBasis().setEulerYPR(yaw, pitch, roll);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(camera_in_map, pose);
    sample_poses_.poses.push_back(pose);
  }

  bool localizeCb(kidnapped_robot::RecognizePlace::Request& request, kidnapped_robot::RecognizePlace::Response& response)
  {
    collect_new_data_ = false;
    best_inliers_ = 0;
    collectImages(num_samples_);
    
    if (best_inliers_ < pr_inliers_) {
      ROS_WARN("Localization failed: best match had only %i inliers, need %i", best_inliers_, pr_inliers_);
      return false;
    }

    tf::poseStampedTFToMsg(best_pose_, response.pose);
    response.inliers = best_inliers_;

    // Publish to initial pose topics
    geometry_msgs::PoseWithCovarianceStamped pwcs;
    pwcs.header = response.pose.header;
    pwcs.pose.pose = response.pose.pose;
    pwcs.pose.covariance[0] = pwcs.pose.covariance[7] = 0.1; // 10cm?
    pwcs.pose.covariance[1] = pwcs.pose.covariance[6] = 0.01;
    pwcs.pose.covariance[27] = 0.1; // yaw variance
    initial_pose_pub_.publish(pwcs);

    geometry_msgs::PoseStamped ps;
    ps.header = pwcs.header;
    ps.pose = pwcs.pose.pose;
    initial_pose_viz_pub_.publish(ps);
    
    return true;
  }
};

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "kidnapped_localization_node");

  if (argc < 5) {
    printf("Usage: %s places.db vocab.tree vocab.weights calonder.rtc\n", argv[0]);
    return 1;
  }

  KidnappedNode node(argv[1], argv[2], argv[3], argv[4]);

  ros::spin();
}
