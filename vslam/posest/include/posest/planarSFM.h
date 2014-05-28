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

#if !defined(_PLANAR_SFM_H)
#define _PLANAR_SFM_H

#include <posest/pe.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace pe
{

/** Implements the algorithm for structure from motion on planar data
  * from the paper by Faugeras and Lustman "Motion and Structure from Motion
  * in a Piecewise Planar Environment, PRAI(2), 1988, pp. 485-508
*/
//! @param intrinsics Intrinsics matrix
//! @param set1 First set of keypoints
//! @param set2 Second set of keypoints
//! @param indices Mapping from the first set to the second
//! @param H Output 3x3 homography matrix
//! @param R Output 3x3 rotation matrix
//! @param T Output 3x1 translation matrix
//! @param reprojectionError Reprojection error for homography used to find inliers.
void planarSFM(const cv::Mat& intrinsics, const std::vector<cv::KeyPoint>& set1, const std::vector<cv::KeyPoint>& set2, const std::vector<int>& indices,
               cv::Mat& H, cv::Mat& R, cv::Mat& T, double reprojectionError = 6.0);

double SFMwithSBA(const cv::Mat& intrinsics, const std::vector<cv::KeyPoint>& points1, const std::vector<cv::KeyPoint>& points2,
        const std::vector<int>& indices, cv::Mat& rvec, cv::Mat& T, double reprojectionError);

double SFMwithSBA(const cv::Mat& intrinsics, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2,
        cv::Mat& rvec, cv::Mat& T, double reprojectionError);

double SFM(const cv::Mat& intrinsics, const std::vector<cv::KeyPoint>& set1, const std::vector<cv::KeyPoint>& set2, const std::vector<int>& indices,
                cv::Mat& R, cv::Mat& T, double reprojectionError = 6.0);

double SFM(const cv::Mat& intrinsics, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
        cv::Mat& R, cv::Mat& T, double reprojectionError = 6.0);

double avgSampsonusError(const cv::Mat& essential, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                double max_error = 1.0, bool verbose = false);

cv::Mat calcEssentialMatrix(const cv::Mat& intrinsics_inv, const cv::Mat& R, const cv::Mat& T);

inline cv::Point3f crossProduct(cv::Point3f& p1, cv::Point3f& p2)
{
    cv::Point3f p3(p1.y*p2.z - p1.z*p2.y, p1.z*p2.x - p1.x*p2.z, p1.x*p2.y - p1.y*p2.x);
    return p3;
};

void dumpFltMat(const char* name, const cv::Mat& mat);

void keyPoints2Point2f(const cv::vector<cv::KeyPoint>& src, std::vector<cv::Point2f>& dst);

void reprojectPoints(const cv::Mat& intrinsics, const cv::Mat& R, const cv::Mat& T, const std::vector<cv::Point2f>& p1,
    const std::vector<cv::Point2f>& p2, std::vector<cv::Point3f>& p, std::vector<bool>& valid);

void matchesFromIndices(const std::vector<cv::KeyPoint>& _set1, const std::vector<cv::KeyPoint>& _set2, const std::vector<int>& indices,
                        std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);

void matchesFromIndices(const std::vector<cv::KeyPoint>& src1, const std::vector<cv::KeyPoint>& src2, const std::vector<cv::DMatch>& indices,
    std::vector<cv::Point2f>& dst1, std::vector<cv::Point2f>& dst2);

float calcScaledPointCloudDistance(const std::vector<cv::Point3f>& points1, const std::vector<cv::Point3f>& points2);

void sba(const cv::Mat& intrinsics, cv::Mat& rvec, cv::Mat& tvec, std::vector<cv::Point3f>& points,
    const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2);

template<class T>
void filterVector(const std::vector<T>& src, const std::vector<bool>& valid, std::vector<T>& dst)
{
    dst.clear();
    for(size_t i = 0; i < valid.size(); i++)
    {
        if(valid[i])
        {
            dst.push_back(src[i]);
        }
    }
};

template<class T>
void filterVector(const std::vector<T>& src, const std::vector<int>& valid, std::vector<T>& dst)
{
    dst.clear();
    for(size_t i = 0; i < valid.size(); i++)
    {
        if(valid[i])
        {
            dst.push_back(src[i]);
        }
    }
};

template<class T>
void filterVector(std::vector<T>& v, const std::vector<bool>& valid)
{
    std::vector<T> out;
    filterVector(v, valid, out);
    v = out;
};

void computeEpipolarInliers(const cv::Mat& essential, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
    std::vector<bool>& inliers, double maxError = 3.0);

void findNaNPoints(const std::vector<cv::Point3f>& points, std::vector<bool>& valid);

void filterInliers(const std::vector<cv::Point3f>& obj_pts, const std::vector<cv::Point2f>& img1_pts,
    const std::vector<cv::Point2f>& img2_pts, const cv::Mat& R, const cv::Mat& T, const cv::Mat& intrinsics,
    double reprojectionError, std::vector<bool>& valid);

void keyPoints2Point2f(const std::vector<cv::KeyPoint>& src, std::vector<cv::Point2f>& dst);

template<class T>
void vectorSubset(const std::vector<T>& src, const std::vector<int>& indices, std::vector<T>& dst)
{
    dst.clear();
    for(size_t i = 0; i < indices.size(); i++)
    {
        dst.push_back(src[indices[i]]);
    }
};

void sample(int max_index, int count, std::vector<int>& sample_indices);

cv::Point3f mult(const cv::Mat& M, const cv::Point3f& p);

void calcRelativeRT(const cv::Mat& R1, const cv::Mat& T1, const cv::Mat& R2, const cv::Mat& T2,
                    cv::Mat& dR, cv::Mat& dT);

}


#endif //_PLANAR_SFM_H
