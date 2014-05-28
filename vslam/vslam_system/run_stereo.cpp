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
#include <features_2d/features_2d.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>

namespace f2d = features_2d;
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

  // Calonder tree setup, don't count in computation time
  cout << "Setting up Calonder tree..." << flush;
  static const char TREES[] = "/u/mihelich/ros/pkgs/wg-ros-pkg-trunk/stacks/visual_feature_detectors/calonder_descriptor/current.rtc";
  f2d::CalonderDescriptorExtractor<float> *cd = new f2d::CalonderDescriptorExtractor<float>(TREES);
  cout << "done" << endl;

  // set up match structures

  // Detect keypoints
  // this returns no keypoints on my sample images...
  // boost::shared_ptr<f2d::FeatureDetector> detector( new f2d::StarFeatureDetector(16, 100) );
  //  boost::shared_ptr<f2d::FeatureDetector> detector( new f2d::StarFeatureDetector );
  //  boost::shared_ptr<f2d::FeatureDetector> detector( new f2d::SurfFeatureDetector(4000.0) );
  //  boost::shared_ptr<f2d::FeatureDetector> detector( new f2d::HarrisFeatureDetector(300, 5) );
  boost::shared_ptr<f2d::FeatureDetector> detector( new f2d::FastFeatureDetector(50) );
  std::vector<cv::KeyPoint> keypoints1, keypoints2;

  // descriptors
  boost::shared_ptr<f2d::DescriptorExtractor> extractor(cd); // this is the calonder descriptor
  //  boost::shared_ptr<f2d::DescriptorExtractor> extractor( new f2d::SurfDescriptorExtractor(3, 4, true) );
  cv::Mat descriptors1, descriptors2;

  // matcher
  boost::shared_ptr<f2d::DescriptorMatcher> matcher(new f2d::BruteForceMatcher< f2d::L2<float> >);
  std::vector<f2d::Match> matches;

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


      // get keypoints
      double t0 = mstime();
      keypoints1.clear();
      keypoints2.clear();
      detector->detect(image1, keypoints1);
      detector->detect(image2, keypoints2);
      double t1 = mstime();
      printf("%d keypoints in ref image\n", (int)keypoints1.size());
      printf("%d keypoints in new image\n", (int)keypoints2.size());

      // Compute descriptors
      extractor->compute(image1, keypoints1, descriptors1);
      extractor->compute(image2, keypoints2, descriptors2);
      double t2 = mstime();

      printf("%d keypoints remaining in ref image\n", (int)keypoints1.size());
      printf("%d keypoints remaining new image\n", (int)keypoints2.size());

      // Perform matching
      matches.clear();
      matcher->matchWindowed(keypoints1, descriptors1, keypoints2, descriptors2, 64, 32, matches);
      double t3 = mstime();
      printf("%d matches\n", (int)matches.size());

      printf("Detector: %0.2f ms;  Descriptor: %0.2f ms;  Matcher: %0.2f ms\n", 
	     t1-t0, t2-t1, t3-t2);


      //
      // do pose estimation
      //

      srand(mstime());

      // set up frames
      Frame f0, f1;
      f0.setCamParams(camp);        // this sets the projection and reprojection matrices
      f1.setCamParams(camp);
      f0.img = image1;
      f0.imgRight = image1r;
      f1.img = image2;
      f1.imgRight = image2r;
      f0.kpts = keypoints1;
      f1.kpts = keypoints2;

      // set stereo on keypoints
      f0.setStereoPoints(64);      // search range
      f1.setStereoPoints(64);
      int n0=0, n1=0;
      for (int i=0; i<(int)f0.goodPts.size(); i++)
	if (f0.goodPts[i]) n0++;
      for (int i=0; i<(int)f1.goodPts.size(); i++)
	if (f1.goodPts[i]) n1++;
      printf("%d/%d, %d/%d good stereo points\n", n0, (int)f0.goodPts.size(), 
	     n1, (int)f1.goodPts.size());

      // save matches
      std::vector<f2d::Match> ms = matches;

      // do pose estimation
      pe::PoseEstimator pe(500,true,2.0e-1,6.0,6.0);
      int inl = pe.estimate(f0,f1,matches,true);
      double t4 = mstime();
      printf("\n");
      printf("Number of inliers: %d; RANSAC+polish time is %0.2f ms\n", inl, t4-t3);
      printf("Total time: %0.2f ms\n", t4-t0);

      cout << pe.trans.transpose().start(3) << endl << endl;
      //      cout << pe.rot << endl;


      // Visualize
      cv::Mat display;
      //  f2d::drawMatchesFlow(image1, keypoints1, image2, keypoints2, matches, display);
      //  f2d::drawMatches(image1, keypoints1, image2, keypoints2, matches, display);
      f2d::drawMatchesFlow(image1, keypoints1, image2, keypoints2, matches, ms, display);
      cv::imshow(window_name, display);
      cv::waitKey(10);

    }
  return 0;
}
