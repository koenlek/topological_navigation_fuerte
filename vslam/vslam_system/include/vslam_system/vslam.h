#ifndef VSLAM_SYSTEM_VSLAM_H
#define VSLAM_SYSTEM_VSLAM_H

#include <vslam_system/vo.h>
#include <vslam_system/place_recognizer.h>
#include <posest/pe3d.h>
#include <posest/peh.h>
#include <sba/sba.h>
#include <frame_common/frame.h>

namespace vslam {

/// \brief VSLAM class that performs visual odometry, loop closure through 
/// place recognition, and large-scale optimization using SBA.
class VslamSystem
{
  public: /// @todo Leave these protected
    frame_common::FrameProc frame_processor_; ///< Frame processor for new frames.
    /// \brief Image frames in system.
    std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> > frames_;
    sba::SysSBA sba_;	 ///< Large-scale SBA system.
    vslam::voSt vo_;   ///< Visual odometry processor.
    vslam::PlaceRecognizer place_recognizer_; ///< Place recognizer
#ifndef HOWARD
    pe::PoseEstimator3d pose_estimator_;      ///< For place recognition matches
#else
    pe::PoseEstimatorH pose_estimator_;
#endif
    
    /// Pointer to pointcloud processor.
    boost::shared_ptr<frame_common::PointcloudProc> pointcloud_processor_;
    /// Pointcloud matches.
    std::vector<cv::DMatch> pointcloud_matches_;
    bool doPointPlane;         // true if we want to process point clouds

  public:
    /// \brief Constructor for VslamSystem.
    /// \param vocab_tree_file Path to the vocabulary tree file.
    /// \param vocab_weights_file Path to the vocabulary tree weights file.
    /// \param min_keyframe_distance Minimum distance between keyframes, in meters.
    /// \param min_keyframe_angle Minimum angle between keyframes, in radians.
    /// \param min_keyframe_inliers Minimum inliers in keyframes.
    VslamSystem(const std::string& vocab_tree_file, const std::string& vocab_weights_file,
                int min_keyframe_inliers=0, double min_keyframe_distance=0.2, 
                double min_keyframe_angle=0.1);

    /// \brief Add a frame to the system.
    /// \param camera_parameters Camera parameters for the cameras.
    /// \param left Left image.
    /// \param right Right image.
    /// \param nfrac Fractional disparity. If above 0, then right is an int16_t 
    ///              disparity image.
    /// \param setPointCloud.  True if point cloud is to be set up from disparities
    bool addFrame(const frame_common::CamParams& camera_parameters,
                  const cv::Mat& left, const cv::Mat& right, int nfrac = 0,
                  bool setPointCloud = false);
    
    /// \brief Add a frame to the system.
    /// \param camera_parameters Camera parameters for the cameras.
    /// \param left Left image.
    /// \param right Right image.
    /// \param ptcloud Pointcloud to add to the frame.
    /// \param nfrac Fractional disparity. If above 0, then right is an int16_t 
    ///              disparity image.
    bool addFrame(const frame_common::CamParams& camera_parameters,
             const cv::Mat& left, const cv::Mat& right, 
             const pcl::PointCloud<pcl::PointXYZRGB>& ptcloud, int nfrac = 0);
             
    void addKeyframe(frame_common::Frame& next_frame);

    /// \brief Perform a refinement on the large-scale SBA system.
    /// \param initial_runs How many iterations to do SBA for.
    void refine(int initial_runs=3);

    int prInliers;  ///< Number of inliers needed for PR match.
    int numPRs;			///< Number of place recognitions that succeeded.
    int nSkip;      ///< Number of the most recent frames to skip for PR checking.

    // parameters settings
    void setPlaceInliers(int n) { prInliers = n; }; ///< Place recognition inliers.
    void setPRSkip(int n) { nSkip = n; };           ///< Set number of keyframes to skip for Place Recognition.
    void setKeyInliers(int n) { vo_.mininls = n; }; ///< Set keyframe inliers.
    void setKeyDist(double n) { vo_.mindist = n; }; ///< Set minimum keyframe distance in meters.
    void setKeyAngle(double n) { vo_.minang = n; }; ///< Set minimum keyframe angle in radians.
    void setNDisp(int n) { frame_processor_.ndisp = n; }; ///< Set the number of disparities in stereo processing.
    void setPRRansacIt(int n) { pose_estimator_.numRansac = n; }; ///< Set the number of RANSAC iterations for place recognition pose estimate.
    void setPRPolish(bool n) { pose_estimator_.polish = n; }; ///< Set whether to polish the place recognition pose estimate using SBA.
    void setVORansacIt(int n) { vo_.pose_estimator_->numRansac = n; }; ///< Set the number of RANSAC iterations for visual odometry pose estimate.
    void setVOPolish(bool n) { vo_.pose_estimator_->polish = n; }; ///< Set whether to polish pose estimate for visual odometry using SBA.
    void setPRWindow(int x, int y) { pose_estimator_.wx = x; pose_estimator_.wy = y; }; ///< Set the window size for place recognition matching.
    void setVOWindow(int x, int y) { vo_.pose_estimator_->wx = x; vo_.pose_estimator_->wy = y; }; ///< Set the window size for place recognition matching.
    void setHuber(double x) { sba_.huber = x; vo_.sba.huber = x; }
    
    
    
    /// \brief Set the pointcloud processor.
    void setPointcloudProc(boost::shared_ptr<frame_common::PointcloudProc> proc)
    {
      pointcloud_processor_ = proc;
      vo_.pointcloud_proc_ = proc;
    }
};

} // namespace vslam

#endif
