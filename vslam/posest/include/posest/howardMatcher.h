#ifndef _HOWARDMATCHER_H_
#define _HOWARDMATCHER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <frame_common/frame.h>

namespace pe
{
  class HowardDescriptorExtractor : public cv::DescriptorExtractor
  {
  public:
      HowardDescriptorExtractor(int _neighborhoodSize = 7);
      virtual void read(const cv::FileNode &fn);
      virtual void write(cv::FileStorage &fs) const;
      virtual int descriptorSize() const;
      virtual int descriptorType() const;

  protected:
      int neighborhoodSize;
      virtual void computeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;
  };

  class HowardStereoMatcher
  {
  public:
    HowardStereoMatcher(float thresh, int descriptorSize);
    void match(const frame_common::Frame& prevFrame, const frame_common::Frame& frame,
               std::vector<cv::DMatch>& matches, std::vector<int>& filteredIndices, const cv::Mat& mask);
  private:
    void filterKpts(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, bool orientation);
    void calculateScoreMatrix(cv::Mat& scoreMatrix);
    void calculateCrossCheckMatches(const cv::Mat& scoreMatrix, std::vector<cv::DMatch>& matches);
    double calcDeltaL(const cv::Point3f& p11, const cv::Point3f& p21, double t, double f, double threshold);

    void calculateConsistMatrix(const std::vector<cv::DMatch>& matches, const frame_common::Frame& prevFrame,
                                const frame_common::Frame& frame, cv::Mat& consistMatrix);
    void filterMatches(const cv::Mat& consistMatrix, std::vector<int>& filteredIndices);
  private:
    float threshold;
    int descriptorSize;
    cv::Mat prevFrameDtors, frameDtors;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    cv::Mat windowedMask;
  };
}

#endif
