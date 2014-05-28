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

#include <vslam_system/vslam_ptcloud.h>
#include <cstdio>

// parameters
//  max distance, angle between keyframes
//  min inliers between keyframes
static const double MAX_KEYFRAME_DISTANCE = 0.0;//0.2;           // meters
static const double MAX_KEYFRAME_ANGLE    = 0.0;//0.1;          // radians  
static const int    MIN_KEYFRAME_INLIERS  = 0;           // depends on number of points, no?

using namespace sba;

namespace vslam {

VslamSystem::VslamSystem(const std::string& vocab_tree_file, const std::string& vocab_weights_file)
  : frame_processor_(10), 
    vo_(boost::shared_ptr<pe::PoseEstimator>(new pe::PoseEstimator3d(1000,true,6.0,8.0,8.0)),
        40, 10, MIN_KEYFRAME_INLIERS, MAX_KEYFRAME_DISTANCE, MAX_KEYFRAME_ANGLE) // 40 frames, 10 fixed
{
  sba_.useCholmod(true);
  vo_.pose_estimator_->windowed = false;
}

bool VslamSystem::addFrame(const frame_common::CamParams& camera_parameters,
                           const cv::Mat& left, const cv::Mat& right, 
                           const pcl::PointCloud<PointXYZRGB>& ptcloud, int nfrac)
{
  // Set up next frame and compute descriptors
  frame_common::FrameExtended next_frame;
  next_frame.setCamParams(camera_parameters); // this sets the projection and reprojection matrices
  frame_processor_.setStereoFrame(next_frame, left, right, nfrac);
  next_frame.frameId = sba_.nodes.size(); // index
  next_frame.img = cv::Mat();   // remove the images
  next_frame.imgRight = cv::Mat();
  next_frame.setPointcloud(ptcloud);
  
  printf("[Pointcloud] Added a pointcloud with %d points.\n", (int)next_frame.cloud.points.size());

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

void VslamSystem::refine()
{
  /// @todo Make these arguments parameters?
  sba_.doSBA(3, 1.0e-4, 1);
  if (sba_.calcRMSCost() > 4.0)
    sba_.doSBA(10, 1.0e-4, 1);  // do more
  if (sba_.calcRMSCost() > 4.0)
    sba_.doSBA(10, 1.0e-4, 1);  // do more
}

} // namespace vslam
