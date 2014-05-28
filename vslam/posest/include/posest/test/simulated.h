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

#ifndef PE_SIMULATED_H_
#define PE_SIMULATED_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <frame_common/frame.h>

namespace pe
{
void generateData(cv::Mat& intrinsics, cv::Mat& R, cv::Mat& T, std::vector<cv::KeyPoint>& points1,
                  std::vector<cv::KeyPoint>& points2, std::vector<int>& indices, std::vector<cv::Point3f>& points);

void generateProjections(const cv::Mat& intrinsics, const cv::Mat& rvec, const cv::Mat& tvec,
                         const std::vector<cv::Point3f>& cloud, std::vector<cv::KeyPoint>& keypoints);
void generatePlanarObject(std::vector<cv::Point3f>& points, cv::Point3f N = cv::Point3f(0.0f, 0.0f, 1.0f), float d = 10.0f);
void generate3DPointCloud(std::vector<cv::Point3f>& points, cv::Point3f pmin = cv::Point3f(-1, -1, 5), cv::Point3f pmax = cv::Point3f(1, 1, 10));

void addLinkNoise(std::vector<cv::DMatch>& indices, double ratio = 0.05);

void calcVisible(const cv::Mat& intrinsics, const cv::Mat& R, const cv::Mat& T,
        const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, std::vector<bool>& visible);

void generateCube(std::vector<cv::Point3f>& cloud);
void generateRing(std::vector<cv::Point3f>& cloud, cv::Point3f center = cv::Point3f(0, 0, 0));

class CameraSimulator
{
public:
//  CameraSimulator(const cv::Mat& intrinsics) {};
//  ~CameraSimulator() {};

  virtual void getNextFrame(std::vector<cv::KeyPoint>& imagePoints, std::vector<cv::DMatch>& matches) = 0;
};

class CircleCameraSimulator : public CameraSimulator
{
public:
  CircleCameraSimulator(const cv::Mat& intrinsics, const std::vector<cv::Point3f>& cloud);
  ~CircleCameraSimulator() {};

  void getNextFrame(std::vector<cv::KeyPoint>& imagePoints, std::vector<cv::DMatch>& matches);
  void calcMatches(const std::vector<int>& newVisible, std::vector<cv::DMatch>& matches);

  virtual void initRT();
  virtual void updateRT();

protected:
  cv::Mat intrinsics_;
  std::vector<cv::Point3f> cloud_;
  std::vector<int> visible_;
  cv::Mat rvec_, tvec_;
  float radius_;
  float angle_;
};

};


#endif /* PE_SIMULATED_H_ */
