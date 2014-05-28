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

#ifndef VSLAM_SYSTEM_VO_PTCLOUD_H
#define VSLAM_SYSTEM_VO_PTCLOUD_H

//
// A class for visual odometry
// Moves a sliding window along a trajectory
//

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <posest/pe3d.h>
#include <posest/pe2d.h>
#include <sba/sba.h>
#include <frame_common/frame_extended.h>
#include <cstdio>

namespace fc  = frame_common;

namespace vslam
{
  class voSt
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    // initialization
    voSt(boost::shared_ptr<pe::PoseEstimator> pose_estimator, int ws=20, int wf=5, int mininls=800, double maxd=0.1, double maxa=0.1);

    /// holds VO window
    sba::SysSBA sba;                 // holds VO window
    
    /// external point index
    /// for relating <sba> points to external points
    vector<int> ipts;

    /// size of window in frames
    int wsize;
    /// number of fixed frames at beginning of window
    int wfixed;

    /// criteria for adding a frame
    double maxdist;
    double maxang;
    int    mininls;

    /// add a new frame
    /// assumes f0 has been set up already, with keypoints and descriptors
    /// returns true if frame is added
    bool addFrame(fc::FrameExtended &f1); 

    /// removes oldest frame from the system
    void removeFrame();

    /// transfers frames to external system
    void transferLatestFrame(std::vector<fc::FrameExtended, Eigen::aligned_allocator<fc::FrameExtended> > &frames,
                             sba::SysSBA &sba);

    /// gets the transform between frames
    int findTransform(int frameId, int n, Vector4d &trans, Quaterniond &qrot, Matrix<double,6,6> &prec);

    /// finds node index of associated frame
    int findNode(int frameId);

    /// previous frames
    std::vector<fc::FrameExtended, Eigen::aligned_allocator<fc::FrameExtended> > frames;

    /// pose estimator
    boost::shared_ptr<pe::PoseEstimator> pose_estimator_;
    
  private:
    std::vector<cv::DMatch> pointcloud_matches_;
  };

  // adds projections between two frames based on inlier matches
  // <ndi0> and <ndi1> are the corresponding node indices in the sba system
  // ipts is an optional global point vector
  void addProjections(fc::FrameExtended &f0, fc::FrameExtended &f1, 
                      std::vector<fc::FrameExtended, Eigen::aligned_allocator<fc::FrameExtended> > &frames,
                      sba::SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w, int ndi0, int ndi1, std::vector<int>* ipts = NULL);
                      
  void addPointCloudProjections(fc::FrameExtended &f0, fc::FrameExtended &f1, 
                    sba::SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w_frame0, 
                      const Matrix<double,3,4>& f2w_frame1, 
                      int ndi0, int ndi1, std::vector<int>* ipts = NULL);

  /// substitutes tri0 for tri1 in a point reference vector
  void substPointRef(std::vector<int> &ipts, int tri0, int tri1);
  
  /// \brief Get a Vector3d projection from a keypoint at index.
  Vector3d getProjection(fc::FrameExtended &frame, int index);


} // end namespace vslam

#endif
