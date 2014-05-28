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

#include <opencv2/core/core.hpp>
#include <posest/test/simulated.h>
#include <posest/planarSFM.h>
#include <iostream>
#include <stdio.h>

using namespace pe;
using namespace cv;

int main(int argc, char** argv)
{
    Mat intrinsics, R0, T0;
    vector<Point3f> points;
    vector<KeyPoint> points1, points2;
    vector<int> indices;
    generateData(intrinsics, R0, T0, points1, points2, indices, points);

    Mat R, T, H;
    double error = SFM(intrinsics, points1, points2, indices, R, T);
    printf("SFM completed with reprojection error %f\n", error);
    T = T*10.0;
    dumpFltMat("SFM returned R", R);
    dumpFltMat("SFM return T", T);

    vector<Point2f> _points1, _points2;
    matchesFromIndices(points1, points2, indices, _points1, _points2);

    Mat essential = calcEssentialMatrix(intrinsics.inv(), R, T);
    vector<bool> inliers;
    computeEpipolarInliers(essential, _points1, _points2, inliers, 5.0);
    filterVector(_points1, inliers);
    filterVector(_points2, inliers);
    filterVector(points, inliers);

    vector<Point3f> cloud;
    vector<bool> valid;
    reprojectPoints(intrinsics, R, T, _points1, _points2, cloud, valid);

    printf("%d points before filtering\n", (int)points.size());
    filterVector(_points1, valid);
    filterVector(_points2, valid);
    filterVector(points, valid);
    filterVector(cloud, valid);

    printf("%d points left after filtering\n", (int)points.size());
    float error0 = calcScaledPointCloudDistance(cloud, points);

    Mat rvec;
    Rodrigues(R, rvec);

    sba(intrinsics, rvec, T, cloud, _points1, _points2);
    findNaNPoints(cloud, valid);
    filterVector(_points1, valid);
    filterVector(_points2, valid);
    filterVector(cloud, valid);
    filterVector(points, valid);

    float error1 = calcScaledPointCloudDistance(cloud, points);

    dumpFltMat("rvec", rvec);
    dumpFltMat("tvec", T);

    printf("%d points left after sba\n3D error after SFM: %f\n 3D error after sba: %f\n",
        points.size(), error0, error1);

    FileStorage fs("extrinsics.yml", FileStorage::WRITE);
    fs << "rvec" << rvec << "T" << T;
}
