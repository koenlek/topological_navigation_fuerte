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

#ifndef VSLAM_SYSTEM_VO_H
#define VSLAM_SYSTEM_VO_H

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
#include <frame_common/frame.h>
#include <cstdio>

namespace fc  = frame_common;

namespace vslam
{
  /// \brief Stereo visual odometry class. Keeps track of a certain size window 
  /// of latest frames in the system, estimates pose changes between frames,
  /// and optimizes the window using SBA.
  class voSt
  {
  public:
    /// \brief Constructor for voSt class.
    /// \param pose_estimator pointer to PoseEstimator object to use for matching frames.
    /// \param ws     Window size for VO's internal SBA system; that is, how many frames to keep track of at a time.
    /// \param wf     Window fixed size; how many of the first frames in the system to keep fixed.
    /// \param mininls Minimum number of point inliers to keep a frame.
    /// \param mind   Minimum linear distance between keyframes (meters).
    /// \param mina   Minimum angular distance between keyframes (radians).
    voSt(boost::shared_ptr<pe::PoseEstimator> pose_estimator, int ws=20, int wf=5, int mininls=800, double mind=0.1, double mina=0.1);

    /// \brief VO's internal SBA system, which holds the VO window.
    sba::SysSBA sba;
    
    /// \brief External point index, for relating internal #sba points to external points.
    vector<int> ipts;

    int wsize; ///< Size of window in frames.
    int wfixed; ///< Number of fixed frames at the beginning of the window.

    // criteria for adding a frame
    double mindist; ///< Minimum linear distance between keyframes (meters).
    double minang;  ///< Minimum angular distance between keyframes (radians).
    int    mininls; ///< Minimum number of inliers.

    /// \brief Add a new frame to the system, if it is a keyframe.
    /// \param fnew The frame to be added.
    /// \return Whether the frame was added as a keyframe.
    bool addFrame(const fc::Frame &fnew); 

    /// \brief Removes oldest frame from the system.
    void removeFrame();

    /// \brief Transfers frames to external sba system.
    /// \param eframes A vector of external frames. The last frame in this will be transferred.
    /// \param esba    External SBA system to add the frame to.
    void transferLatestFrame(std::vector<fc::Frame, Eigen::aligned_allocator<fc::Frame> > &eframes,
                             sba::SysSBA &esba);

    /// \brief Gets the transform between frames.
    /// \param frameId ID of the first frame.
    /// \param n       Number of frames ahead the second frame is (frameId + n).
    /// \param trans   Translation between the two frames (meters).
    /// \param qrot    Rotation between the two frames.
    /// \return        Frame ID of the second frame.
    int findTransform(int frameId, int n, Vector4d &trans, Quaterniond &qrot, Matrix<double,6,6> &prec);

    /// Finds #sba node index of associated frame.
    int findNode(int frameId);

    /// Previous frames in the system.
    std::vector<fc::Frame, Eigen::aligned_allocator<fc::Frame> > frames;

    /// Pointer to pose estimator object.
    boost::shared_ptr<pe::PoseEstimator> pose_estimator_;
    
    /// Pointer to pointcloud processor.
    boost::shared_ptr<frame_common::PointcloudProc> pointcloud_proc_;
    bool doPointPlane;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment
  
  private:
    /// Pointcloud matches.
    std::vector<cv::DMatch> pointcloud_matches_;
  };

  /// \brief Adds projections between two frames based on inlier matches.
  /// Adds any points not previously present in the external system, and fills
  /// in ipts.
  /// \param f0      Previous frame.
  /// \param f1      Newest frame to add.
  /// \param frames  All the frames in the system.
  /// \param sba     SBA system to add the frames to.
  /// \param inliers Matches between f0 and f1; indices of the points and
  ///                projections to be added to the system.
  /// \param f2w     Transform between the frame f0 to world coordinates.
  /// \param ndi0    Node index of the old frame (f0) in SBA.
  /// \param ndi1    Node index of the new frame (f1) in SBA.
  /// \param ipts    Optional parameter for a mapping between internal and 
  ///                external indices.
  void addProjections(fc::Frame &f0, fc::Frame &f1, 
                      std::vector<fc::Frame, Eigen::aligned_allocator<fc::Frame> > &frames,
                      sba::SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w, int ndi0, int ndi1, std::vector<int>* ipts = NULL);
  
  /// \brief Adds point-plane projections between two frames based on inlier matches.
  /// Adds any points not previously present in the external system, and fills
  /// in ipts.
  /// \param f0      Previous frame.
  /// \param f1      Newest frame to add.
  /// \param sba     SBA system to add the frames to.
  /// \param inliers Matches between f0 and f1; indices of the points and
  ///                projections to be added to the system.
  /// \param f2w_frame0 Transform between the frame f0 to world coordinates.
  /// \param f2w_frame1 Transform between the frame f1 to world coordinates.
  /// \param ndi0    Node index of the old frame (f0) in SBA.
  /// \param ndi1    Node index of the new frame (f1) in SBA.
  /// \param ipts    Optional parameter for a mapping between internal and 
  ///                external indices.                    
  void addPointCloudProjections(fc::Frame &f0, fc::Frame &f1, 
                  sba::SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                    const Matrix<double,3,4>& f2w_frame0, 
                    const Matrix<double,3,4>& f2w_frame1, 
                    int ndi0, int ndi1, std::vector<int>* ipts = NULL);
  
  /// \brief substitutes tri0 for tri1 in a point reference vector.
  void substPointRef(std::vector<int> &ipts, int tri0, int tri1);
  
  /// \brief Get a Vector3d projection from a keypoint at index.
  Vector3d getProjection(fc::Frame &frame, int index);
} // end namespace vslam

#endif
