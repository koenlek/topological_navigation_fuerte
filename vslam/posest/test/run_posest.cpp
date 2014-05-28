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


#include <posest/pe3d.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>


using namespace frame_common;
using namespace std;

// elapsed time in milliseconds
#include <sys/time.h>
static double mstime()
{
  timeval tv;
  gettimeofday(&tv,NULL);
  long long ts = tv.tv_sec;
  ts *= 1000000;
  ts += tv.tv_usec;
  return (double)ts*.001;
}


int main(int argc, char** argv)
{
  if (argc < 6)
    {
      printf("Args are: <params> <left0> <left1> <right0> <right1>\n");
      exit(0);
    }

  // get camera parameters, in the form: fx fy cx cy tx
  fstream fstr;
  fstr.open(argv[1],fstream::in);
  if (!fstr.is_open())
    {
      printf("Can't open camera file %s\n",argv[1]);
      exit(0);
    }
  CamParams camp;
  fstr >> camp.fx;
  fstr >> camp.fy;
  fstr >> camp.cx;
  fstr >> camp.cy;
  fstr >> camp.tx;

  cout << "Cam params: " << camp.fx << " " << camp.fy << " " << camp.cx
       << " " << camp.cy << " " << camp.tx << endl;

  // Load images
  cv::Mat image1 = cv::imread(argv[2], 0);
  cv::Mat image2 = cv::imread(argv[3], 0);
  cv::Mat image1r = cv::imread(argv[4], 0);
  cv::Mat image2r = cv::imread(argv[5], 0);
  printf("Image size: %d x %d\n", image1.cols, image1.rows);
  printf("Image size: %d x %d\n", image1r.cols, image1r.rows);
  printf("Image size: %d x %d\n", image2.cols, image2.rows);
  printf("Image size: %d x %d\n", image2r.cols, image2r.rows);

  if (image1.rows == 0 || image1r.rows == 0 || image2.rows == 0 || image2r.rows == 0)
    exit(0);

  cout << "Setting up frame processing..." << flush;
  FrameProc fp(30);
  cout << "done" << endl;
  // set up frames
  Frame f0, f1;
  f0.setCamParams(camp);        // this sets the projection and reprojection matrices
  f1.setCamParams(camp);

  double t0 = mstime();
  fp.setStereoFrame(f0,image1,image1r);
  fp.setStereoFrame(f1,image2,image2r);

  printf("%d keypoints from %s\n", (int)f0.kpts.size(), argv[2]);
  printf("%d keypoints from %s\n", (int)f1.kpts.size(), argv[3]);

  //
  // do pose estimation
  //

  srand(mstime());		// for RANSAC

  int n0=0, n1=0;
  for (int i=0; i<(int)f0.goodPts.size(); i++)
    if (f0.goodPts[i]) n0++;
  for (int i=0; i<(int)f1.goodPts.size(); i++)
    if (f1.goodPts[i]) n1++;
  printf("%d/%d, %d/%d good stereo points\n", n0, (int)f0.goodPts.size(), 
         n1, (int)f1.goodPts.size());

  for (int i=0; i<100; i++)
    {
      // do pose estimation
      pe::PoseEstimator3d pe(500,true,10.0,4.0,4.0);
      double t0 = mstime();
      int inl = pe.estimate(f0,f1);
      double t1 = mstime();
      printf("\n");
      printf("%d matches\n", (int)pe.matches.size());
      printf("Number of inliers: %d; time is %0.2f ms\n", inl, t1-t0);

      cout << pe.trans.transpose() << endl << endl;
      cout << pe.rot << endl << "==========================" << endl;


      // Visualize
      cv::Mat display;
      drawMatches(image1, f0.kpts, image2, f1.kpts, pe.inliers, display);
      //  f2d::drawMatchesFlow(image1, f0.kpts, image2, f1.kpts, pe.inliers, pe.matches, display);
      const std::string window_name = "matches";
      cv::namedWindow(window_name,0);
      cv::imshow(window_name, display);
      cv::waitKey(2000);
    }
  return 0;
}
