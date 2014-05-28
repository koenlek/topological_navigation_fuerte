#include <vslam_system/vslam_mono.h>

using namespace sba;

namespace vslam 
{

  VslamSystemMono::VslamSystemMono(const std::string& vocab_tree_file, const std::string& vocab_weights_file)
    : VslamSystem(vocab_tree_file, vocab_weights_file)
  {
    vo_.pose_estimator_ = boost::shared_ptr<pe::PoseEstimator>(new pe::PoseEstimator2d);
  }
  
  bool VslamSystemMono::addFrame(const frame_common::CamParams& camera_parameters,
                           const cv::Mat& image)
  {
    // Set up next frame and compute descriptors
    frame_common::Frame next_frame;
    next_frame.setCamParams(camera_parameters); // this sets the projection and reprojection matrices
    frame_processor_.setMonoFrame(next_frame, image);
    next_frame.frameId = sba_.nodes.size(); // index
    next_frame.img = cv::Mat();   // remove the images
    next_frame.imgRight = cv::Mat();

    // Add frame to visual odometer
    bool is_keyframe = vo_.addFrame(next_frame);

    // grow full SBA
    if (is_keyframe) 
    {
      frames_.push_back(next_frame);
      vo_.transferLatestFrame(frames_, sba_);
    }
      
    if (frames_.size() > 1 && vo_.pose_estimator_->inliers.size() < 40)
      std::cout << std::endl << "******** Bad image match: " << std::endl << std::endl;

    return is_keyframe;
  }
  
  } // vslam
