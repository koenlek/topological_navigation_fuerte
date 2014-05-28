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


//
// drawing VO traces
//

#include <frame_common/frame.h>
#include <frame_common/frame_extended.h>

using namespace std;

namespace frame_common {

  static void drawSmallKeypoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints,
                                 cv::Mat& display, cv::Scalar color)
  {
    int shift_bits = 4;
    int multiplier = 1 << shift_bits;
    for (std::vector<cv::KeyPoint>::const_iterator i = keypoints.begin(), ie = keypoints.end();
         i != ie; ++i) {
      cv::Point center(i->pt.x * multiplier, i->pt.y * multiplier);
      int radius = 1*multiplier;
      cv::circle(display, center, radius, color, 1, CV_AA, shift_bits);
      /// @todo Draw orientation
    }
  }

  void drawVOtracks(const cv::Mat &image,
                    const vector<Frame, Eigen::aligned_allocator<Frame> > &frames, 
                    cv::Mat &display)
  {
    // Set up image display
    display = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    int shift_bits = 4;
    int multiplier = 1 << shift_bits;

    // Last frame
    const Frame &f0 = frames.back();

    // Draw keypoints
    cv::cvtColor(image, display, CV_GRAY2BGR);
    drawSmallKeypoints(image, f0.kpts, display, CV_RGB(255, 0, 0));
    drawSmallKeypoints(image, f0.tkpts, display, CV_RGB(0, 255, 0));

    return;

    if (frames.size() < 2) return;

    // make a map of the points referred to in the last frame
    const vector<int> &ipts = f0.ipts;
    map<int,int> pmap;
    for (int i=0; i<(int)ipts.size(); i++)
      if (ipts[i] >= 0)
        pmap.insert(pair<int,int>(ipts[i],i));
    
    // iterate over frames, drawing tracks
    for (int fi=0; fi<(int)frames.size(); fi++)
      {
        int c = ((frames.size()-1-fi)*255*2)/(frames.size()-1);
        if (c > 255) c=255;
        cv::Scalar color(255,c,0); // BGR order

        const Frame &f1 = frames[fi];
        const vector<int> &iipts = f1.ipts;
        map<int,int>::iterator it;
        for (int i=0; i<(int)iipts.size(); i++)
          {
            int k1 = iipts[i];
            it = pmap.find(k1);
            if (it != pmap.end())     // found a match!
              {
                int k0 = it->second;
                const cv::KeyPoint& kpt0 = f0.kpts[k0];
                const cv::KeyPoint& kpt1 = f1.kpts[i];
                cv::Point center1(kpt0.pt.x * multiplier, kpt0.pt.y * multiplier);
                cv::Point center2(kpt1.pt.x * multiplier, kpt1.pt.y * multiplier);
                cv::line(display, center1, center2, color, 1, CV_AA, shift_bits);
                pmap.erase(it);
              }
          }
      }

  }
  
  void drawVOtracks(const cv::Mat &image,
                    const vector<FrameExtended, Eigen::aligned_allocator<FrameExtended> > &frames, 
                    cv::Mat &display)
  {
    // Set up image display
    display = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    int shift_bits = 4;
    int multiplier = 1 << shift_bits;

    // Last frame
    const Frame &f0 = frames.back();

    // Draw keypoints
    cv::cvtColor(image, display, CV_GRAY2BGR);
    drawSmallKeypoints(image, f0.kpts, display, CV_RGB(255, 0, 0));

    if (frames.size() < 2) return;

    // make a map of the points referred to in the last frame
    const vector<int> &ipts = f0.ipts;
    map<int,int> pmap;
    for (int i=0; i<(int)ipts.size(); i++)
      if (ipts[i] >= 0)
        pmap.insert(pair<int,int>(ipts[i],i));
    
    // iterate over frames, drawing tracks
    for (int fi=0; fi<(int)frames.size(); fi++)
      {
        int c = ((frames.size()-1-fi)*255*2)/(frames.size()-1);
        if (c > 255) c=255;
        cv::Scalar color(255,c,0); // BGR order

        const Frame &f1 = frames[fi];
        const vector<int> &iipts = f1.ipts;
        map<int,int>::iterator it;
        for (int i=0; i<(int)iipts.size(); i++)
          {
            int k1 = iipts[i];
            it = pmap.find(k1);
            if (it != pmap.end())     // found a match!
              {
                int k0 = it->second;
                const cv::KeyPoint& kpt0 = f0.kpts[k0];
                const cv::KeyPoint& kpt1 = f1.kpts[i];
                cv::Point center1(kpt0.pt.x * multiplier, kpt0.pt.y * multiplier);
                cv::Point center2(kpt1.pt.x * multiplier, kpt1.pt.y * multiplier);
                cv::line(display, center1, center2, color, 1, CV_AA, shift_bits);
                pmap.erase(it);
              }
          }
      }

  }



} //namespace frame_common
