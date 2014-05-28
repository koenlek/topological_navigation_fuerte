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

// Messages
#include <sba/Frame.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/legacy/legacy.hpp>

// parameters
//  max distance, angle between keyframes
//  min inliers between keyframes
static const double MAX_KEYFRAME_DISTANCE = 0.2;           // meters
static const double MAX_KEYFRAME_ANGLE    = 0.1;          // radians
static const int    MIN_KEYFRAME_INLIERS  = 0;           // depends on number of points, no?

using namespace vslam;

class VONode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
                                    
  // Publications for SBA Node
  ros::Publisher sba_frames_pub_;
  
  // Publications for visualization
  image_transport::CameraPublisher vo_tracks_pub_;
  cv::Mat vo_display_;
 
  // Processing state
  sensor_msgs::CvBridge l_bridge_, r_bridge_;
  image_geometry::StereoCameraModel cam_model_;
  cv::Ptr<cv::FeatureDetector> detector_;

  frame_common::FrameProc frame_processor_;
  /// stereo image key frames in system
  std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> > frames_;
  vslam::voSt vo_; /// VO processor
  
  int numframes;
  int numpoints;
  int numcameras;
  
  vector<sba::Projection> proj_msgs;
  vector<sba::WorldPoint> point_msgs;
  vector<sba::CameraNode> node_msgs;
  
  // Reconfigure server (use the same one as Stereo VSLAM for now)
  typedef dynamic_reconfigure::Server<vslam_system::StereoVslamNodeConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  
  bool init;
  
public:
  // Code copied from stereo_vslam_node.cpp
  VONode(const std::string& vocab_tree_file, const std::string& vocab_weights_file,
         const std::string& calonder_trees_file)
    : it_(nh_), sync_(3),
      detector_(new vslam_system::AnyDetector),
      vo_(boost::shared_ptr<pe::PoseEstimator>(new pe::PoseEstimator3d(1000,true,6.0,8.0,8.0)),
          40, 10, MIN_KEYFRAME_INLIERS, MAX_KEYFRAME_DISTANCE, MAX_KEYFRAME_ANGLE),
          numframes(0), numpoints(0), numcameras(0), init(true)
  {
    // Use calonder descriptor
    typedef cv::CalonderDescriptorExtractor<float> Calonder;
    frame_processor_.setFrameDescriptor(new Calonder(calonder_trees_file));
    
    // Synchronize inputs
    l_image_sub_.subscribe(it_, "left/image_rect", 1);
    l_info_sub_ .subscribe(nh_, "left/camera_info", 1);
    r_image_sub_.subscribe(it_, "right/image_rect", 1);
    r_info_sub_ .subscribe(nh_, "right/camera_info", 1);
    vo_tracks_pub_ = it_.advertiseCamera("/vo/vo_tracks/image", 1);
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&VONode::imageCb, this, _1, _2, _3, _4) );
    
    // Advertise topics for connection to SBA node
    // sba_nodes_pub_ = nh_.advertise<sba::CameraNode>("sba_nodes", 5000);
    // sba_points_pub_ = nh_.advertise<geometry_msgs::PointStamped>("sba_points", 5000);
    sba_frames_pub_ = nh_.advertise<sba::Frame>("/sba/frames", 5000);
    
    // Dynamic reconfigure for parameters
    ReconfigureServer::CallbackType f = boost::bind(&VONode::configCb, this, _1, _2);
    reconfigure_server_.setCallback(f);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
               const sensor_msgs::CameraInfoConstPtr& l_cam_info,
               const sensor_msgs::ImageConstPtr& r_image,
               const sensor_msgs::CameraInfoConstPtr& r_cam_info)
  {
    ROS_INFO("In callback, seq = %u", l_cam_info->header.seq);
    
    // Convert ROS messages for use with OpenCV
    cv::Mat left, right;
    try
    {
      left  = l_bridge_.imgMsgToCv(l_image, "mono8");
      right = r_bridge_.imgMsgToCv(r_image, "mono8");
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
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

    if (addFrame(cam_params, left, right))
    {
      if (vo_tracks_pub_.getNumSubscribers() > 0) 
      {
        frame_common::drawVOtracks(left, vo_.frames, vo_display_);
        IplImage ipl = vo_display_;
        sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
        msg->header = l_cam_info->header;
        vo_tracks_pub_.publish(msg, l_cam_info);
      }
    }
  }
  
  void configCb(vslam_system::StereoVslamNodeConfig& config, uint32_t level)
  {
    dynamic_cast<vslam_system::AnyDetector*>((cv::FeatureDetector*)detector_)->update(config);
    frame_processor_.detector = detector_;

    /*vslam_system_.setPRRansacIt(config.pr_ransac_iterations);
    vslam_system_.setPRPolish(config.pr_polish);
    vslam_system_.setVORansacIt(config.vo_ransac_iterations);
    vslam_system_.setVOPolish(config.vo_polish);*/
  }
  
  bool addFrame(const frame_common::CamParams& camera_parameters,
                           const cv::Mat& left, const cv::Mat& right)
  {
    // Set up next frame and compute descriptors
    frame_common::Frame next_frame;
    next_frame.setCamParams(camera_parameters); // this sets the projection and reprojection matrices
    frame_processor_.setStereoFrame(next_frame, left, right);
    next_frame.frameId = numframes++; // index
    next_frame.img = cv::Mat();   // remove the images
    next_frame.imgRight = cv::Mat();

    // Add frame to visual odometer
    bool is_keyframe = vo_.addFrame(next_frame);

    // grow full SBA
    if (is_keyframe)
    {
      frames_.push_back(next_frame);
      publishLatestFrame();
    }
    
    return is_keyframe;
  }
  
  void publishLatestFrame()
  {
    if (sba_frames_pub_.getNumSubscribers() > 0) 
    {
      // Clear the message vectors.
      proj_msgs.clear();
      node_msgs.clear();
      point_msgs.clear();
      
      /*sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
      msg->header = l_cam_info->header;
      vo_tracks_pub_.publish(msg, l_cam_info);*/
      
      // latest frame
      frame_common::Frame &f1 = frames_.back();

      // set up point indices to NULL
      f1.ipts.assign(f1.kpts.size(), -1);

      // get RW position given relative position
      Quaterniond fq;
      Vector4d trans;
      Matrix<double,3,4> f2w;
      
      if (init)                   // use zero origin
        {
          trans = Vector4d(0,0,0,1);
          fq.coeffs() = Vector4d(0,0,0,1);
          fq.normalize();
        }
      else
        {
          // Just use the pose of the latest added frame to VO's internal sba.
          sba::Node &nd0 = *(vo_.sba.nodes.end()-1);
          sba::Node &nd1 = vo_.sba.nodes.back();
          fq = nd1.qrot;
          trans = nd1.trans;
          
          sba::transformF2W(f2w,nd0.trans,nd0.qrot);
          // trans.head(3) = f2w*trans;
        }

      // Publish node for the frame, setting it to fixed if we're initializing,
      // floating otherwise.
      int cameraindex = publishNode(trans, fq, f1.cam, init);
      
      if (init)
      {
        init = false;
        return;
      }

      /// TODO this also assume most recent VO operation was a keyframe
      /// should reconstruct inliers from most recent two frames
      
      /// Also assumes that last node was published from this node as well
      frame_common::Frame &f0 = *(frames_.end()-2);
      addProjections(f0, f1, vo_.pose_estimator_->inliers, f2w, cameraindex-1, cameraindex);
      
      publishFrame();
    }
  }
  
  void publishFrame()
  {
    sba::Frame msg;
    msg.nodes = node_msgs;
    msg.points = point_msgs;
    msg.projections = proj_msgs;
    
    sba_frames_pub_.publish(msg);
  }
  
  int publishNode(Eigen::Matrix<double,4,1> trans, 
                      Eigen::Quaternion<double> fq,
                      const frame_common::CamParams &cp,
                      bool isFixed)
  {
    sba::CameraNode msg;
    
    msg.index = numcameras;
    
    msg.transform.translation.x = trans.x();
    msg.transform.translation.y = trans.y();
    msg.transform.translation.z = trans.z();
    msg.transform.rotation.x = fq.x();
    msg.transform.rotation.y = fq.y();
    msg.transform.rotation.z = fq.z();
    msg.transform.rotation.w = fq.w();
    msg.fx = cp.fx;
    msg.fy = cp.fy;
    msg.cx = cp.cx;
    msg.cy = cp.cy;
    msg.baseline = cp.tx;
    msg.fixed = isFixed;
    
    node_msgs.push_back(msg);
    
    return numcameras++;
    //sba_nodes_pub_.publish(msg);
  }
  
  int publishPoint(sba::Point pt)
  {
    sba::WorldPoint msg;
    
    msg.index = numpoints;
    
    msg.x = pt.x();
    msg.y = pt.y();
    msg.z = pt.z();
    msg.w = pt.w();
    
    point_msgs.push_back(msg);
    
    return numpoints++;
    
    //sba_points_pub_.publish(msg);
  }
  
  void publishProjection(int ci, int pi, Eigen::Vector3d &q, bool stereo)
  {
    sba::Projection msg;
    msg.camindex = ci;
    msg.pointindex = pi;
    msg.u = q.x();
    msg.v = q.y();
    msg.d = q.z();
    msg.stereo = stereo;
    msg.usecovariance = false;
    
    proj_msgs.push_back(msg);
  }
  
    // add connections between frames, based on keypoint matches
  void addProjections(fc::Frame &f0, fc::Frame &f1, 
                      const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w, int ndi0, int ndi1)
  {
    // set up array to kill duplicate matches
    vector<bool> matched0(f0.ipts.size(),0);
    vector<bool> matched1(f1.ipts.size(),0);

    // add points and projections
    for (int i=0; i<(int)inliers.size(); i++)
      {
        int i0 = inliers[i].queryIdx;
        int i1 = inliers[i].trainIdx;

        if (matched0[i0]) continue;
        if (matched1[i1]) continue;
        matched0[i0] = true;
        matched1[i1] = true;

        int pti;

        if (f0.ipts[i0] < 0 && f1.ipts[i1] < 0)    // new point
          {
            Vector4d pt;
            pt.head(3) = f2w*f0.pts[i0]; // transform to RW coords
            pt(3) = 1.0;
            
            pti = publishPoint(pt);
            f0.ipts[i0] = pti;
            f1.ipts[i1] = pti;

            vo_.ipts.push_back(-1);  // external point index

            Vector3d ipt = getProjection(f0, i0);
            publishProjection(ndi0, pti, ipt, true);

            // projected point, ul,vl,ur
            ipt = getProjection(f1, i1);
            publishProjection(ndi1, pti, ipt, true);
          }
          
        // Commented out right now since we have no good way to merge tracks
        else if (f0.ipts[i0] >= 0 && f1.ipts[i1] >= 0) // merge two tracks
          {
            if (f0.ipts[i0] != f1.ipts[i1]) // different tracks
              {
                // if (sba_merge_tracks_client_.isValid()) // Do we need to do this instead?
                
                printf("We are trying to merge tracks %d and %d with ipts %d and %d\n", i0, i1, f0.ipts[i0], f1.ipts[i1]);
                
                /*sba::MergeTracks::Request req;
                sba::MergeTracks::Response res;
                req.trackid0 = f0.ipts[i0];
                req.trackid1 = f1.ipts[i1];
               
                if (sba_merge_tracks_client_.call(req, res))
                {
                  int tri = res.trackid;
                  if (tri >= 0)   // successful merge
                    {
                      // update the ipts in frames that connect to this track
                      for (int i = 0; i < numframes; i++)
                        {
                          if (tri == f0.ipts[i0])
                            sba::substPointRef(frames_[numframes].ipts, tri, f1.ipts[i1]);
                          else
                            sba::substPointRef(frames_[numframes].ipts, tri, f0.ipts[i0]);
                        }
                    }
                  } */ 
              }
          }

        else if (f1.ipts[i1] < 0)                 // add to previous point track
          {
            pti = f0.ipts[i0];
            f1.ipts[i1] = pti;
          
            // projected point, ul,vl,ur
            Vector3d ipt = getProjection(f1, i1);
            publishProjection(ndi1, pti, ipt, true);
          }
        else if (f0.ipts[i0] < 0)                 // add to previous point track
          {
            pti = f1.ipts[i1];
            f0.ipts[i0] = pti;
          
            // projected point, ul,vl,ur
            Vector3d ipt = getProjection(f0, i0);
            publishProjection(ndi0, pti, ipt, true);
          }
      }
  }
  
}; // class VO Node

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_vo");
  if (argc < 4) {
    printf("Usage: %s <vocab tree file> <vocab weights file> <calonder trees file>\n", argv[0]);
    return 1;
  }
  
  VONode vonode(argv[1], argv[2], argv[3]);
  ros::spin();
}

