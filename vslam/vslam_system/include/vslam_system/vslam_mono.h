#ifndef VSLAM_SYSTEM_VSLAM_MONO_H
#define VSLAM_SYSTEM_VSLAM_MONO_H

#include <vslam_system/vslam.h>

namespace vslam {

/// \brief VSLAM class that performs visual odometry, loop closure through 
/// place recognition, and large-scale optimization using SBA on monocular images.
class VslamSystemMono : public VslamSystem
{
  public:
    /// \brief Constructor for VslamSystemMono.
    /// \param vocab_tree_file Path to the vocabulary tree file.
    /// \param vocab_weights_file Path to the vocabulary tree weights file.
    VslamSystemMono(const std::string& vocab_tree_file, const std::string& vocab_weights_file);

    /// \brief Add a monocular frame to the system.
    /// \param camera_parameters Camera parameters for the cameras.
    /// \param image Image to add.
    bool addFrame(const frame_common::CamParams& camera_parameters,
                  const cv::Mat& image);
};

} // namespace vslam

#endif
