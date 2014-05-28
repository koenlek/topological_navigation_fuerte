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
#include "sba/sba_file_io.h"
using namespace Eigen;
using namespace sba;

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/time.h>

using namespace std;

// Bring in gtest
#include <gtest/gtest.h>

//
// tolerance for floating-point tests
//

double tol = 1e-4;
#define EXPECT_EQ_TOL(a,b) { if (fabs((a)-(b)) > max(fabs(a),fabs(b))*tol) EXPECT_DOUBLE_EQ(a,b); }
#define EXPECT_EQ_ABS(a,b,t) { if (fabs((a)-(b)) > (t)) EXPECT_DOUBLE_EQ(a,b); }


// This is for calculating JtJ for a 2-node system
#if 0
TEST(TestSBA, SpiralSystem_2nodes_A)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 2*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.0, 0.0); // image noise (pixels), frame noise (meters)

  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  sba.printStats();

  sba.nFixed = 1;               // one fixed frame
  sba.doSBA(0);
  sba.setupSys(0.0);

  sba.writeA("N2.5d.txt");

  cout << endl << "[TestSBA] Wrote JtJ for 2 nodes" << endl << endl;
}

TEST(TestSBA, SpiralSystem_2nodes2_A)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 10.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 2*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.0, 0.0); // image noise (pixels), frame noise (meters)

  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  sba.printStats();

  sba.nFixed = 1;               // one fixed frame
  sba.doSBA(0);
  sba.setupSys(0.0);

  sba.writeA("N2.10d.txt");

  cout << endl << "[TestSBA] Wrote JtJ for 2 nodes" << endl << endl;
}

TEST(TestSBA, SpiralSystem_2nodes3_A)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 15.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 2*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.0, 0.0); // image noise (pixels), frame noise (meters)

  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  sba.printStats();

  sba.nFixed = 1;               // one fixed frame
  sba.doSBA(0);
  sba.setupSys(0.0);

  sba.writeA("N2.15d.txt");

  cout << endl << "[TestSBA] Wrote JtJ for 2 nodes" << endl << endl;
}

TEST(TestSBA, SpiralSystem_2nodes_vert_A)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 360.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 2*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.0, 0.0); // image noise (pixels), frame noise (meters)

  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  sba.printStats();

  sba.nFixed = 1;               // one fixed frame
  sba.doSBA(0);
  sba.setupSys(0.0);

  sba.writeA("N2V.txt");

  cout << endl << "[TestSBA] Wrote JtJ for 2 nodes" << endl << endl;
}


#endif


// test the transform functions
TEST(TestSBA, SpiralSystem_init90n)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 20*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.2, 0.01); // image noise (pixels), frame noise (meters)

  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  sba.printStats();

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-3); // should be within 15mm

  // find number of projection measurements
  int nms = 0;
  for (int i=0; i<(int)sba.tracks.size(); i++)
    nms += sba.tracks[i].projections.size();
  double cost = sba.calcCost();
  cost = sqrt(cost/nms);
  EXPECT_EQ_ABS(cost,0.0,0.01); // RMS pixel error should be low
}


// initial angle 0.0
TEST(TestSBA, SpiralSystem_init0)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = true;

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, 0.0, 20*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.0, 0.05, 0.01); // image noise (pixels), frame noise (meters)

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;

  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  double cost = sba.calcCost();
  cout << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.01); // squared cost should be low, but it's not
}

// initial angle 0.0
TEST(TestSBA, SpiralSystem_global_init0)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = false;

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, 0.0, 20*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.0, 0.05, 0.01); // image noise (pixels), frame noise (meters)

  // check isValid flag
  //  (sba.tracks[0])[0].isValid = false;

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  double cost = sba.calcCost();
  cout << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.01); // squared cost should be low
}


// initial angle 0.0
TEST(TestSBA, SpiralSystem_init90)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 2.5, 10.0, // system, saved initial positions, near, far
               0.6, kfang, M_PI/2, 20*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.0, 0.05, 0.01); // image noise (pixels), frame noise (meters)

  // check isValid flag
  //  (sba.tracks[0])[0].isValid = false;

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-3); // should be within 15mm

  double cost = sba.calcCost();
  cout << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,100.0); // squared cost should be low, but this test often fails...
}



// initial angle 180
// local angles don't work very well here
// seems to have trouble going through the singularity
TEST(TestSBA, SpiralSystem_init180)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  spiral_setup(sba, cpars, cps, 0.5, 10.0, // system, saved initial positions, near, far
               0.6, kfang, M_PI, 20*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.0, 0.05, 0.001); // image noise (pixels), frame noise (meters)

  // check isValid flag
  //  (sba.tracks[0])[0].isValid = false;

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-3); // should be within 5mm

  double cost = sba.calcCost();
  cout << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.01); // squared cost should be low
}

// initial angle 180
TEST(TestSBA, SpiralSystem_global_init180)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = false;

  spiral_setup(sba, cpars, cps, 0.5, 10.0, // system, saved initial positions, near, far
               0.6, kfang, M_PI, 20*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.0, 0.05, 0.001); // image noise (pixels), frame noise (meters)

  // check isValid flag
  //  (sba.tracks[0])[0].isValid = false;

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  double cost = sba.calcCost();
  cout << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.01); // squared cost should be low
}


// test the transform functions
TEST(TestSBA, SpiralSystem_global_init90n)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = false;

  spiral_setup(sba, cpars, cps, 2.0, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 20*kfang/360.0, // point density, angle per frame, initial angle, number of cycles (frames),
               0.0, 0.2, 0.1); // image noise (pixels), frame noise (meters)


  // check isValid flag
  //  (sba.tracks[0])[0].isValid = false;
  cout << "[Spiral] Testing global angles" << endl;
  cout << "[Spiral] Initial cost is " << sba.calcCost() << endl;

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;

  sba.nFixed = 1;               // one fixed frame
  sba.printStats();

  long long t0, t1;
  t0 = utime();
  int niters = sba.doSBA(20);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-3); // should be within 15mm

  // find number of projection measurements
  int nms = 0;
  for (int i=0; i<(int)sba.tracks.size(); i++)
    nms += sba.tracks[i].projections.size();
  double cost = sba.calcCost();
  cost = sqrt(cost/nms);
  EXPECT_EQ_ABS(cost,0.0,0.01); // RMS pixel error should be low
}


// large system test
TEST(TestSBA, SpiralSystem_global_400)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = false;    // check with global angles

  spiral_setup(sba, cpars, cps, 2.5, 10.0, // system, saved initial positions, near, far
               0.6, kfang, -M_PI/2, 400*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.5, 0.1, 0.01); // image noise (pixels), frame noise (meters), angle noise (qs)


  sba.printStats();

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters = sba.doSBA(1);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  double dist = 100000.0;
  int bad = sba.removeBad(dist);
  int gone = sba.reduceTracks();

  cout << "[SBAsys] Removed " << bad << " projections with distance > " << dist << endl;
  cout << "[SBAsys] Reduced " << gone << " tracks" << endl;
  niters = sba.doSBA(5);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  // find number of projection measurements
  int nms = 0;
  for (int i=0; i<(int)sba.tracks.size(); i++)
    nms += sba.tracks[i].projections.size();

  double cost = sba.calcCost();
  cost = sqrt(cost/nms);
  cout << "[TestSBA] Final rms pixel error: " << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.5); // squared cost should be low
}


// large system test
TEST(TestSBA, SpiralSystem_54)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 10.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = true;    // use incremental form

  spiral_setup(sba, cpars, cps, 2.0, 5.0, // system, saved initial positions, near, far
               3.0, kfang, -M_PI/2, 54*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.5, 0.1, 0.01); // image noise (pixels), frame noise (meters), angle noise (qs)

  sba.printStats();
  //  cout << "[SBAsys] Writing file" << endl;
  //  sba.writeFile("700cams.txt");

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame

  int niters = sba.doSBA(1);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  // very odd - leaving this out causes a segfault...
  cout << endl << "We're here!!!!!!!!!!!!!!!!!!!!!" << endl << endl;

  // this gives a segfault, need to debug
  double dist = 10000.0;
  int bad = sba.removeBad(dist);
  int gone = sba.reduceTracks();

  cout << "[SBAsys] Removed " << bad << " projections with distance > " << dist << endl;
  cout << "[SBAsys] Reduced " << gone << " tracks" << endl;

  niters = sba.doSBA(10,1.0e-4,true);

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


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
  printf("\n[TestSBA] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  // find number of projection measurements
  int nms = 0;
  for (int i=0; i<(int)sba.tracks.size(); i++)
    nms += sba.tracks[i].projections.size();

  double cost = sba.calcCost();
  cost = sqrt(cost/nms);
  cout << "[TestSBA] Final rms pixel error: " << cost << endl;
  EXPECT_EQ_ABS(cost,0.0,0.5); // squared cost should be low
}

// large system test
TEST(TestSBA, SpiralSystem_400)
{
  // define a camera and bundle system
  SysSBA sba;
  Node::initDr();
  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;

  double kfang = 5.0;
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  sba.useLocalAngles = true;    // use incremental form
  sba.csp.useCholmod = true;

  spiral_setup(sba, cpars, cps, 3.0, 3.5, // system, saved initial positions, near, far
               10.0, kfang, -M_PI/2, 400*kfang/360.0, // point density, angle per frame, 
                                                    // initial angle, number of cycles (frames),
               0.5, 0.1, 0.01); // image noise (pixels), frame noise (meters), angle noise (qs)

  sba.printStats();

#if 1
  cout << "[SBAsys] Writing file" << endl;
  writeLourakisFile("spiral-4000", sba);
#endif

  for (int i=0; i<3; i++)
    cout << "Quaternion: " << sba.nodes[i].qrot.coeffs().transpose() << endl;
  cout << endl;


  long long t0, t1;
  t0 = utime();
  sba.nFixed = 1;               // one fixed frame
  int niters;

#if 0
  niters = sba.doSBA(1);
  t1 = utime();
  printf("[TestSBA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSBA] Accepted iterations: %d\n", niters);

  double dist = 10000.0;
  int bad = sba.removeBad(dist);
  int gone = sba.reduceTracks();

  cout << "[SBAsys] Removed " << bad << " projections with distance > " << dist << endl;
  cout << "[SBAsys] Reduced " << gone << " tracks" << endl;
#endif

  cout << endl << "[SBAsys] Full system" << endl;
  niters = sba.doSBA(10,1.0e-3,1); // full system

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

  EXPECT_EQ_ABS(sqerr,0.0,5e-3); // should be within 5mm

  // find number of projection measurements
  int nms = 0;
  for (int i=0; i<(int)sba.tracks.size(); i++)
    nms += sba.tracks[i].projections.size();

  double cost = sba.calcCost();
  cost = sqrt(cost/nms);
  cout << "[TestSBA] Final rms pixel error: " << cost << endl << endl;
  EXPECT_EQ_ABS(cost,0.0,0.5); // squared cost should be low
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}


