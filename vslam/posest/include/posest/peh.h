#ifndef _PEH_H_
#define _PEH_H_

#include <frame_common/frame.h>
#include <boost/shared_ptr.hpp>
#include <posest/pe.h>
#include <posest/howardMatcher.h>
#include <cv.h>
#include <cstdlib>
#include <math.h>

namespace fc  = frame_common;
using namespace std;
namespace pe
{
class PoseEstimatorH : public PoseEstimator
{
public:
  using PoseEstimator::estimate;

  PoseEstimatorH(int NRansac, bool LMpolish, double mind,
                  double maxidx, double maxidd, float matcherThreshold, int minMatchesCount, int descriptorSize) :
                    PoseEstimator(NRansac, LMpolish, mind, maxidx, maxidd), minMatchesCount(minMatchesCount)
  {
    usedMethod = Stereo;
    howardMatcher = new HowardStereoMatcher(matcherThreshold, descriptorSize);
  };
  ~PoseEstimatorH() { };

  virtual int estimate(const fc::Frame& frame1, const fc::Frame& frame2);

  virtual int estimate(const fc::Frame& frame1, const fc::Frame& frame2,
                       const std::vector<cv::DMatch> &matches);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment
private:
  cv::Ptr<HowardStereoMatcher> howardMatcher;
  int minMatchesCount;
  std::vector<int> filteredIndices;
};

} // ends namespace pe
#endif // _PEH_H_
