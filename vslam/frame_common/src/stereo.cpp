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

#include <frame_common/stereo.h>
#include <xmmintrin.h>

namespace frame_common
{

  // !! Must free memory after using

  uint8_t *grab_16x16(uint8_t *im, int xim, int x, int y)
  { 
    char sub[256];
    if (1) 
      {
        for (size_t i = 0; i < 16; i++)
	        memcpy(sub + 16 * i, im + x + (y + i) * xim, 16);
      } 
    else 
      {
#define COPY16(N)							\
      _mm_storeu_ps((float*)(sub + 16 * N), _mm_loadu_ps((float*)(im + x + (y + N) * xim)));
        COPY16(0)
	      COPY16(1)
	      COPY16(2)
	      COPY16(3)
	      COPY16(4)
	      COPY16(5)
	      COPY16(6)
	      COPY16(7)
	      COPY16(8)
	      COPY16(9)
	      COPY16(10)
	      COPY16(11)
	      COPY16(12)
	      COPY16(13)
	      COPY16(14)
	      COPY16(15)
	    }
    uint8_t *return_val = new uint8_t[257];
    return_val[256] = '\0';
    memcpy(return_val, sub, 256);
    return return_val;
  }

  SparseStereo::SparseStereo(const cv::Mat& leftImg, const cv::Mat& rightImg, 
			     bool use_grad_im, uint max_disparity)
  {
    //    printf("Doing sparse stereo with ndisp = %d\n", max_disparity);

    lim = leftImg;
    rim = rightImg;
    use_grad_img = use_grad_im;
    max_disp = max_disparity;
    if (use_grad_im)
      {
	      uint8_t *buf = new uint8_t[lim.cols * lim.rows];
	      lgrad = new uint8_t[lim.cols * lim.rows];
	      rgrad = new uint8_t[rim.cols*rim.rows];
	      do_prefilter_xsobel(lim.data, lgrad, lim.cols, lim.rows, ftzero, buf );
	      do_prefilter_xsobel(rim.data, rgrad, rim.cols, rim.rows, ftzero, buf );
	      delete [] buf;
      }
    else
      {
	      lgrad = NULL;
	      rgrad = NULL;
      }
  }

  SparseStereo::~SparseStereo()
  {
    delete [] lgrad;
    delete [] rgrad;
  }

  double SparseStereo::lookup_disparity(int x, int y) const
  {
    uint8_t* refpat, *rim_as_arr, *lim_as_arr;
    int w = lim.cols;
    int h = lim.rows;
    if(use_grad_img && lgrad)
      {
	      lim_as_arr = lgrad;
	      rim_as_arr = rgrad;
      }
    else
      {
	      lim_as_arr = lim.data;
	      rim_as_arr = rim.data;
      }
    refpat = grab_16x16(lim_as_arr, w, x-7, y-7);
    int v = do_stereo_sparse(refpat, rim_as_arr, x, y, w, h, ftzero, max_disp, tfilter_thresh, ufilter_thresh);
    free(refpat);
    if (v < 0)
      return 0.0;
    else
      return v*(1.0/16.0);
  }

  // dense stereo
  // more accurate, but takes longer
  // could outfit sparse stereo with same algorithm
  // <frac> is set if we're passing in a stereo disparity image,
  //   it gives the 

  DenseStereo::DenseStereo(const cv::Mat& leftImg, const cv::Mat& rightImg, 
			   int nd, double frac)
  {
    if (nd > 0)			// set number of disparities
      ndisp = nd;
	
    //    printf("Doing dense stereo with ndisp = %d\n", ndisp);

    lim = leftImg;
    rim = rightImg;

    // variables
    int xim = lim.cols;
    int yim = lim.rows;

    // clear disparity buffer - do we need to do this???
    imDisp = (int16_t *)MEMALIGN(xim*yim*2);
    memset(imDisp, 0, xim*yim*sizeof(int16_t));

    // check for 
    if (frac > 0)		// set fractional disparities, if we pass in a
      {				//   disparity map
        fdisp = frac;		
        memcpy(imDisp,rim.data,xim*yim*sizeof(int16_t));
        return;
      }


    // some parameters
    int dlen   = ndisp;         // number of disparities
    int corr   = corrSize;      // correlation window size
    int tthresh = textureThresh; // texture threshold
    int uthresh = uniqueThresh;	// uniqueness threshold, percent
    fdisp = 1.0/16.0;		// fixed for this algorithm

    // allocate buffers
    // TODO: make these consistent with current values

    buf  = (uint8_t *)MEMALIGN(yim*2*dlen*(corr+5)); // local storage for the algorithm
    flim = (uint8_t *)MEMALIGN(xim*yim); // feature image
    frim = (uint8_t *)MEMALIGN(xim*yim); // feature image

    // prefilter
    do_prefilter(lim.data, flim, xim, yim, ftzero, buf);
    do_prefilter(rim.data, frim, xim, yim, ftzero, buf);



    do_stereo(flim, frim, imDisp, NULL, xim, yim,
              ftzero, corr, corr, dlen, tthresh, uthresh, buf);

    MEMFREE(buf);
    MEMFREE(flim);
    MEMFREE(frim);
  }

  DenseStereo::~DenseStereo()
  {
    MEMFREE(imDisp);
  }

  // should look in a small area around x,y
  double DenseStereo::lookup_disparity(int x, int y) const
  {
    int w = lim.cols;
    int h = lim.rows;
    if (imDisp)
      {
        double v = (double)(imDisp[y*w+x]);
        if (v > 0.0)
          return v*fdisp;
      }

    return 0.0;
  }

  int DenseStereo::ndisp = 64;
  int DenseStereo::textureThresh = 4;
  int DenseStereo::uniqueThresh = 28;
  int DenseStereo::corrSize = 11;
  double DenseStereo::fdisp = 1.0/16.0;


} // end namespace frame_common
