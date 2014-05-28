#ifndef VSLAM_SYSTEM_VSLAM_H
#define VSLAM_SYSTEM_VSLAM_H

#include <vslam_system/vo_ptcloud.h>
#include <vslam_system/place_recognizer.h>
#include <posest/pe3d.h>
#include <sba/sba.h>
#include <frame_common/frame.h>

namespace vslam {

class VslamSystem
{
public: /// @todo Leave these protected
  frame_common::FrameProc frame_processor_;
  /// stereo image frames in system
  std::vector<frame_common::FrameExtended, Eigen::aligned_allocator<frame_common::FrameExtended> > frames_;
  sba::SysSBA sba_;			/// SBA system
  vslam::voSt vo_; /// VO processor

public:
  VslamSystem(const std::string& vocab_tree_file, const std::string& vocab_weights_file);

  // <nfrac> > 0 for sending in int16_t disparity image
  bool addFrame(const frame_common::CamParams& camera_parameters,
                const cv::Mat& left, const cv::Mat& right, 
                const pcl::PointCloud<PointXYZRGB>& ptcloud, int nfrac = 0);

  void refine();

  // parameters settings
  void setKeyInliers(int n) { vo_.mininls = n; }; /// keyframe inliers
  void setKeyDist(double n) { vo_.maxdist = n; }; /// keyframe distance, m
  void setKeyAngle(double n) { vo_.maxang = n; }; /// keyframe angle, rads
  void setNDisp(int n) { frame_processor_.ndisp = n; }; /// number of disparities in stereo processing
  void setVORansacIt(int n) { vo_.pose_estimator_->numRansac = n; }; /// number of RANSAC iterations for pose estimate
  void setVOPolish(bool n) { vo_.pose_estimator_->polish = n; }; /// whether to polish pose estimate using SBA

  //size_t size() const;
};

} // namespace vslam

#endif
