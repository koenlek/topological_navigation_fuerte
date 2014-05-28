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


#include <posest/pe.h>
#include <sba/sba.h>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>


using namespace Eigen;
using namespace sba;
using namespace frame_common;
using namespace std;

namespace pe
{
  PoseEstimator::PoseEstimator(int NRansac, bool LMpolish, double mind,
                                   double maxidx, double maxidd) : testMode(false)
  {
    polish = LMpolish;
    numRansac = NRansac;
    minMatchDisp = mind;
    maxInlierXDist2 = maxidx*maxidx;
    maxInlierDDist2 = maxidd*maxidd;
    rot.setIdentity();
    trans.setZero();
    
    // matcher
    matcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2));
    wx = 92; wy = 48;
    windowed = true;
  }

void PoseEstimator::matchFrames(const fc::Frame& f0, const fc::Frame& f1, std::vector<cv::DMatch>& fwd_matches)
  {
    cv::Mat mask;
    if (windowed)
      mask = cv::windowedMatchingMask(f0.kpts, f1.kpts, wx, wy);

    matcher->clear();
    matcher->match(f0.dtors, f1.dtors, fwd_matches, mask); 
  }

  //
  // find the best estimate for a geometrically-consistent match
  //   sets up frames internally using sparse stereo
  // NOTE: we do forward/reverse matching and save all unique matches
  //   sometimes forward works best, sometimes reverse
  // uses the SVD procedure for aligning point clouds
  // final SVD polishing step
  //

  int PoseEstimator::estimate(const Frame &f0, const Frame &f1)
  {
    // set up match lists
    matches.clear();
    inliers.clear();

    // do forward and reverse matches
    std::vector<cv::DMatch> fwd_matches, rev_matches;
    matchFrames(f0, f1, fwd_matches);
    matchFrames(f1, f0, rev_matches);
    //printf("**** Forward matches: %d, reverse matches: %d ****\n", (int)fwd_matches.size(), (int)rev_matches.size());

    // combine unique matches into one list
    for (int i = 0; i < (int)fwd_matches.size(); ++i) {
      if (fwd_matches[i].trainIdx >= 0)
        matches.push_back( cv::DMatch(i, fwd_matches[i].trainIdx, 0.f) );
    }
    for (int i = 0; i < (int)rev_matches.size(); ++i) {
      if (rev_matches[i].trainIdx >= 0 && i != fwd_matches[rev_matches[i].trainIdx].trainIdx)
        matches.push_back( cv::DMatch(rev_matches[i].trainIdx, i, 0.f) );
    }
    //printf("**** Total unique matches: %d ****\n", (int)matches.size());
    
    // do it
    return estimate(f0, f1, matches);
  }

  void PoseEstimator::setMatcher(const cv::Ptr<cv::DescriptorMatcher>& new_matcher)
  {
    matcher = new_matcher;
  }
  
  void PoseEstimator::setTestMode(bool mode)
  {
    testMode = mode;
  };
}
