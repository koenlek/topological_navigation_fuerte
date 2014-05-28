#ifndef VSLAM_SYSTEM_ANY_DETECTOR_H
#define VSLAM_SYSTEM_ANY_DETECTOR_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

namespace vslam_system {

class AnyDetector : public cv::FeatureDetector
{
  typedef cv::Ptr<cv::FeatureDetector> DetectorPtr;
  
  DetectorPtr active_detector_;

public:
  AnyDetector(const DetectorPtr& detector = new cv::FastFeatureDetector)
    : active_detector_(detector)
  {
  }

  /// @todo Remove detect() overload. This is only for backwards-compatibility with cturtle-era OpenCV.
  virtual void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const
  {
    detectImpl(image, keypoints, mask);
  }

  template <class Config>
  void update(Config& config)
  {
    if (config.detector == "FAST") {
      active_detector_ = new cv::FastFeatureDetector(config.fast_threshold, config.fast_nonmax_suppression);
    }
    else if (config.detector == "Harris") {
      active_detector_ = new cv::GoodFeaturesToTrackDetector(config.harris_max_keypoints, config.harris_quality_level,
                                                             config.harris_min_distance, config.harris_block_size,
                                                             true, config.harris_k);
    }
    else if (config.detector == "Star") {
      /// @todo Enforce supported sizes
      active_detector_ = new cv::StarFeatureDetector(config.star_max_size, config.star_response_threshold,
                                                     config.star_line_threshold_projected,
                                                     config.star_line_threshold_binarized,
                                                     config.star_suppress_nonmax_size);
    }
    else if (config.detector == "SURF") {
      active_detector_ = new cv::SurfFeatureDetector(config.surf_hessian_threshold, config.surf_octaves,
                                                     config.surf_octave_layers);
    }
    else
      ROS_ERROR("Unknown detector '%s'", config.detector.c_str());
    /// @todo SIFT, MSER

    if (config.grid_adapter) {
      active_detector_ = new cv::GridAdaptedFeatureDetector(active_detector_, config.grid_max_keypoints,
                                                            config.grid_rows, config.grid_cols);
    }
  }

protected:
  virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask ) const
  {
    active_detector_->detect(image, keypoints, mask);
  }
};

} // namespace vslam_system

#endif
