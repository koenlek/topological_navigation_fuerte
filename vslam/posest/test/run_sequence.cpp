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
// runs a sequence of stereo images into an SBA engine
//

#include <posest/pe3d.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>

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

// Names of left and right files in directory (with wildcards)
char *lreg, *rreg;

// Filters for scandir
int getleft(struct dirent const *entry)
{
  if (!fnmatch(lreg,entry->d_name,0))
    return 1;
  return 0;
}

int getright(struct dirent const *entry)
{
  if (!fnmatch(rreg,entry->d_name,0))
    return 1;
  return 0;
}


int main(int argc, char** argv)
{
  if (argc < 5)
    {
      printf("Args are: <param file> <image dir> <left image file template> <right image file template> \n");
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


  // get left/right image file names, sorted
  char *dir = argv[2];
  lreg = argv[3];
  rreg = argv[4];
  struct dirent **lims, **rims;
  int nlim, nrim;
  nlim = scandir(dir,&lims,getleft,alphasort);
  printf("Found %d left images\n", nlim);
  printf("%s\n",lims[0]->d_name);

  nrim = scandir(argv[2],&rims,getright,alphasort);
  printf("Found %d right images\n", nrim);
  printf("%s\n",rims[0]->d_name);

  if (nlim != nrim)
    {
      printf("Number of left/right images does not match: %d vs. %d\n", nlim, nrim);
      exit(0);
    }

  // set up structures
  cout << "Setting up frame processing..." << flush;
  FrameProc fp;			// can be slow, setting up Calonder tree
  cout << "done" << endl;
  pe::PoseEstimator3d pe(500,true,2.0e-1,6.0,6.0);

  srand(mstime());		// for RANSAC

  // window
  const std::string window_name = "matches";
  cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

  for (int i=0; i<nlim-1; i++)
    {
      // Load images
      char fn[2048];
      sprintf(fn,"%s/%s",dir,lims[i]->d_name);
      cv::Mat image1 = cv::imread(fn,0);
      sprintf(fn,"%s/%s",dir,lims[i+1]->d_name);
      cv::Mat image2 = cv::imread(fn,0);
      sprintf(fn,"%s/%s",dir,rims[i]->d_name);
      cv::Mat image1r = cv::imread(fn,0);
      sprintf(fn,"%s/%s",dir,rims[i+1]->d_name);
      cv::Mat image2r = cv::imread(fn,0);

#if 0
      printf("Image size: %d x %d\n", image1.cols, image1.rows);
      printf("Image size: %d x %d\n", image1r.cols, image1r.rows);
      printf("Image size: %d x %d\n", image2.cols, image2.rows);
      printf("Image size: %d x %d\n", image2r.cols, image2r.rows);
#endif

      if (image1.rows == 0 || image1r.rows == 0 || image2.rows == 0 || image2r.rows == 0)
	exit(0);


      //
      // do pose estimation
      //

      // set up frames
      Frame f0, f1;
      f0.setCamParams(camp);        // this sets the projection and reprojection matrices
      f1.setCamParams(camp);

      fp.setStereoFrame(f0,image1,image1r);
      double t0 = mstime();
      fp.setStereoFrame(f1,image2,image2r);

      int n0=0, n1=0;
      for (int i=0; i<(int)f0.goodPts.size(); i++)
	if (f0.goodPts[i]) n0++;
      for (int i=0; i<(int)f1.goodPts.size(); i++)
	if (f1.goodPts[i]) n1++;
      printf("%d/%d, %d/%d good stereo points\n", n0, (int)f0.goodPts.size(), 
	     n1, (int)f1.goodPts.size());

      // do pose estimation
      double t3 = mstime();
      int inl = pe.estimate(f0,f1);
      double t4 = mstime();
      printf("Number of inliers: %d; RANSAC+polish time is %0.2f ms\n", inl, t4-t3);
      printf("Total time: %0.2f ms\n", t4-t0);

      cout << "Translation: " << pe.trans.transpose().head(3) << endl << endl;
      //      cout << pe.rot << endl;


      // Visualize
      /// @todo Restore visualization. drawMatchesFlow not implemented here
#if 0
      cv::Mat display;
      //  f2d::drawMatchesFlow(image1, keypoints1, image2, keypoints2, matches, display);
      //  f2d::drawMatches(image1, keypoints1, image2, keypoints2, matches, display);
      f2d::drawMatchesFlow(image1, f0.kpts, image2, f1.kpts, pe.inliers, pe.matches, display);
      cv::imshow(window_name, display);
      cv::waitKey(10);
#endif
    }
  return 0;
}
