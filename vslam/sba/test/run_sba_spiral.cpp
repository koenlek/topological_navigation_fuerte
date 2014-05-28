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

// test using a spiral trajectory
// not using gtest here

#include "sba/sba.h"
#include "sba/sba_setup.h"
using namespace Eigen;
using namespace sba;

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/time.h>

using namespace std;


#define SAVE_RESULTS
//#define SAVE_BUNDLER

int main(int argc, char **argv)
{
  // file for saving results
#ifdef SAVE_RESULTS
  FILE *fd = fopen("results.txt","a");
#endif


  int nncams = 4000;
  int bcams = 100;
  double pdensity = 10.0;
  double pinterval = 0.5;

  if (argc <= 1)
    printf("Args are: <start> <end> <pt density 10.0> <pt interval 0.5>\n\n");

  if (argc > 1)
    bcams = atoi(argv[1]);

  if (argc > 2)
    nncams = atoi(argv[2]);

  if (argc > 3)
    pdensity = atof(argv[3]);

  if (argc > 4)
    pinterval = atof(argv[4]);

  char name[2048];

  for (int ncams = bcams; ncams <= nncams; ncams = (int)((double)ncams*1.5))
    {
      // define a camera and bundle system
      SysSBA sba;
      vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;
      cps.clear();

      double kfang = 5.0;
      CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

      sba.useLocalAngles = true;    // use incremental form
      sba.csp.useCholmod = true;

      spiral_setup(sba, cpars, cps, 3.0, 3.0+pinterval, // system, saved initial positions, near, far
                   pdensity, kfang, -M_PI/2, ncams*kfang/360.0, // point density, angle per frame, 
                   // initial angle, number of cycles (frames),
                   0.5, 0.1, 0.01); // image noise (pixels), frame noise (meters), angle noise (qs)

      //  sba.printStats();

      // average track size
      int npts = sba.tracks.size();
      double tsize = 0;             // average track size
      for (int i=0; i<npts; i++)
        tsize += sba.tracks[i].projections.size();
      tsize = tsize/(double)npts;

      // matrix connections
      int nprjs = 0;
      MatrixXi conns;
      conns.setIdentity(ncams,ncams);
      for (int i=0; i<(int)sba.tracks.size(); i++)
        {
          ProjMap &prjs = sba.tracks[i].projections;
          nprjs += prjs.size();
          if (prjs.size()>1)
            for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
              {
                Proj &prj = itr->second;
                for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                  {
                    Proj &prj2 = itr2->second;
                    conns(prj.ndi,prj2.ndi) = 1;
                    conns(prj2.ndi,prj.ndi) = 1;
                  }
              }
        }

      int tot = 0;
      for (int i=0; i<ncams; i++)
        for (int j=0; j<ncams; j++)
          tot += conns(i,j);
      double mfill = (double)tot/(double)(ncams*ncams);

      printf("\nCams: %d  Avg projs: %0.1f  Avg. track size: %0.1f  Percent fill: %0.1f\n\n",
             ncams, (double)nprjs/(double)ncams, tsize, 100.0*mfill);

#ifdef SAVE_RESULTS
      fprintf(fd,"%d %0.1f %0.1f %0.1f ",
              ncams, (double)nprjs/(double)ncams, tsize, 100.0*mfill);
#endif

      conns.setZero(1,1);

#ifdef SAVE_BUNDLER
      sprintf(name,"spiral-%04d-%d",ncams,(int)(10.0*pinterval));
      cout << "[SBAsys] Writing file " << name << endl;
      sba.writeFile(name);
#endif

      for (int i=0; i<3; i++)
        cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
      cout << endl;


      long long t0, t1;
      sba.nFixed = 1;           // one fixed frame
      int niters;
      t0 = utime();
      niters = sba.doSBA(1,1.0e-3,1); // full system
      t1 = utime();


#ifdef SAVE_RESULTS
      fprintf(fd,"%0.1f %0.1f %0.1f %0.1f\n",
              (sba.t2-sba.t1)*0.001,
              (sba.t1-sba.t0)*0.001,
              (sba.t3-sba.t2)*0.001,
              (t1-t0)*0.001);
      fflush(fd);
#endif

      // test results
      double sqerr = 0.0;
      for (int i=0; i<(int)cps.size(); i++)
        {
          Matrix<double,6,1> &cp = cps[i]; // old camera pose
          Vector3d tp = cp.head(3);
          Vector3d tpn = sba.nodes[i].trans.head(3);
          //      printf("\n[TestSBA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
          //      printf("[TestSBA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);
          Vector3d err = tp-tpn;
          sqerr += err.squaredNorm();
        }
  
      sqerr = sqerr / (double)(cps.size());
      printf("[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

      // find number of projection measurements
      int nms = 0;
      for (int i=0; i<(int)sba.tracks.size(); i++)
        nms += sba.tracks[i].projections.size();

      double cost = sba.calcCost();
      cost = sqrt(cost/nms);
      cout << "[TestSBA] Final rms pixel error: " << cost << endl << endl;
    } // end of FOR loop over ncams

#ifdef SAVE_RESULTS
  fclose(fd);
#endif


}
