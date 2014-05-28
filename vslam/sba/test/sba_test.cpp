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

// test fixture

// Bring in my package's API, which is what I'm testing
#include "sba/sba.h"
#include "sba/sba_file_io.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace Eigen;
using namespace sba;
using namespace frame_common;

#include <iostream>
#include <fstream>
using namespace std;

#define LOCAL_ANGLES


//
// tolerance for floating-point tests
//

double tol = 1e-4;
#define EXPECT_EQ_TOL(a,b) { if (fabs((a)-(b)) > max(fabs(a),fabs(b))*tol) EXPECT_DOUBLE_EQ(a,b); }
#define EXPECT_EQ_ABS(a,b,t) { if (fabs((a)-(b)) > (t)) EXPECT_DOUBLE_EQ(a,b); }

// test the transform functions
TEST(SBAtest, SimpleSystem)
{
  // set up full system
  SysSBA sys;
  
  // set of world points
  // each row is a point
  Matrix<double,23,4> pts;
  pts << 0.5,  0.2, 3,   1.0,
         1,    0,   2,   1.0,
        -1,    0,   2,   1.0,
         0,    0.2, 3,   1.0,
         1,    1,   2,   1.0,
        -1,   -1,   2,   1.0,
         1,    0.2, 4,   1.0,
         0,    1,   2.5, 1.0,
         0,   -1,   2.5, 1.0,
         0.2,  0,   3,   1.0,
        -1,    1,   2.5, 1.0,
         1,   -1,   2.5, 1.0,
         0.5,  0.2, 4,   1.0,
         0.2, -1.3, 2.5, 1.0,
        -0.5, -1,   2.5, 1.0,
         0.2, -0.7, 3,   1.0,
        -1,    1,   3.5, 1.0,
         1,   -1,   3.5, 1.0,
         0.5,  0.2, 4.6, 1.0,
        -1,    0,   4,   1.0,
         0,    0,   4,   1.0,
         1,    1,   4,   1.0,
        -1,   -1,   4,   1.0;

  for (int i=0; i<pts.rows(); i++)
    {
      Point pt(pts.row(i));
      sys.addPoint(pt);
    }

  Node::initDr();               // set up fixed matrices

  // set of nodes
  // three nodes, one at origin, two displaced
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  Quaternion<double> frq2(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt2(0.2, -0.4, 1.0, 1.0); // frame position in the world
  Quaternion<double> frq3(AngleAxisd(10*M_PI/180,Vector3d(-.2,.1,-.3).normalized())); // frame rotation in the world
  Vector4d frt3(-0.2, 0.4, 1.0, 1.0); // frame position in the world

  // set up nodes
  Node nd1;
  Vector4d qr(0,0,0,1);
  nd1.qrot = qr;		// no rotation 
  nd1.trans = Vector4d::Zero();	// or translation
  nd1.setTransform();		// set up world2node transform
  nd1.setKcam(cpars);		// set up node2image projection
  nd1.setProjection();
#ifdef LOCAL_ANGLES
  nd1.setDr(true);              // set rotational derivatives
#else
  nd1.setDr(false);             // set rotational derivatives
#endif
  nd1.isFixed = true;

  Node nd2;
  nd2.qrot = frq2;	
  cout << "Quaternion: " << nd2.qrot.coeffs().transpose() << endl;
  nd2.trans = frt2;
  cout << "Translation: " << nd2.trans.transpose() << endl << endl;
  nd2.setTransform();		// set up world2node transform
  nd2.setKcam(cpars);		// set up node2image projection
  nd2.setProjection();
#ifdef LOCAL_ANGLES
  nd2.setDr(true);              // set rotational derivatives
#else
  nd2.setDr(false);             // set rotational derivatives
#endif
  nd2.isFixed = false;

  Node nd3;
  nd3.qrot = frq3;	
  cout << "Quaternion: " << nd3.qrot.coeffs().transpose() << endl;
  nd3.trans = frt3;
  cout << "Translation: " << nd3.trans.transpose() << endl << endl;
  nd3.setTransform();		// set up world2node transform
  nd3.setKcam(cpars);		// set up node2image projection
  nd3.setProjection();
#ifdef LOCAL_ANGLES
  nd3.setDr(true);              // set rotational derivatives
#else
  nd3.setDr(false);             // set rotational derivatives
#endif
  nd3.isFixed = false;

  sys.nodes.push_back(nd1);
  sys.nodes.push_back(nd2);
  sys.nodes.push_back(nd3);

  // set up projections onto nodes
  int ind = 0;
  double inoise = 0;//0.5;
  Vector2d n2;

  for(unsigned int i = 0; i < sys.tracks.size(); i++)
    {
      Point pt = sys.tracks[i].point;      
      ProjMap &prjs = sys.tracks[i].projections;	// new point track
      Proj prj;
      prj.isValid = true;
      Vector2d ipt;

      n2.setRandom();
      nd1.project2im(ipt,pt);	// set up projection measurement
      prj.ndi = 0;		// nd1 index
      prj.kp.head<2>() = ipt + n2*inoise;
      prjs[0] = prj;

      n2.setRandom();
      nd2.project2im(ipt,pt);	// set up projection measurement
      prj.ndi = 1;		// nd2 index
      prj.kp.head<2>() = ipt + n2*inoise;
      prjs[1] = prj;

      n2.setRandom();
      nd3.project2im(ipt,pt);	// set up projection measurement
      prj.ndi = 2;		// nd3 index
      prj.kp.head<2>() = ipt + n2*inoise;
      prjs[2] = prj;

      ind++;
    }

  double qnoise = 0;//10*M_PI/180;	// in radians
  double tnoise = 0;//0.05;		// in meters

  // add random noise to node positions
  nd2.qrot.coeffs().head<3>() += qnoise*Vector3d::Random();
  nd2.normRot();
  cout << "Quaternion: " << nd2.qrot.coeffs().transpose() << endl << endl;
  nd2.trans.head<3>() += tnoise*Vector3d::Random();
  nd2.setTransform();		// set up world2node transform
  nd2.setProjection();
#ifdef LOCAL_ANGLES
  nd2.setDr(true);              // set rotational derivatives
#else
  nd2.setDr(false);              // set rotational derivatives
#endif
  sys.nodes[1] = nd2;		// reset node
  
  nd3.qrot.coeffs().head<3>() += qnoise*Vector3d::Random();
  nd3.normRot();
  //  cout << "Quaternion: " << nd3.qrot.transpose() << endl << endl;
  nd3.trans.head<3>() += tnoise*Vector3d::Random();
  nd3.setTransform();		// set up world2node transform
  nd3.setProjection();		// set up node2image projection
#ifdef LOCAL_ANGLES
  nd3.setDr(true);              // set rotational derivatives
#else
  nd3.setDr(false);             // set rotational derivatives
#endif
  sys.nodes[2] = nd3;		// reset node

#if 1
  writeGraphFile("sba.graph", sys);
#endif

#if 0
  // set up system, no lambda for here
  sys.setupSys(0.0);
  ofstream(fd);
  fd.open("A.txt");
  fd.precision(8);		// this is truly inane
  fd << fixed;
  fd << sys.A << endl;
  fd.close();
  fd.open("B.txt");
  fd.precision(8);
  fd << fixed;
  fd << sys.B << endl;
  fd.close();
#endif
  
#ifndef LOCAL_ANGLES
  sys.useLocalAngles = false;
#endif

  sys.nFixed = 1;		// number of fixed cameras
  
  sys.doSBA(10);

  cout << endl << "Quaternion: " << sys.nodes[1].qrot.coeffs().transpose() << endl;
  // normalize output translation
  Vector4d frt2a = sys.nodes[1].trans;
  double s = frt2.head<3>().norm() / frt2a.head<3>().norm();
  frt2a.head<3>() *= s;
  cout << "Translation: " << frt2a.transpose() << endl << endl;

  cout << "Quaternion: " << sys.nodes[2].qrot.coeffs().transpose() << endl;
  Vector4d frt3a = sys.nodes[2].trans;
  s = frt3.head<3>().norm() / frt3a.head<3>().norm();
  frt3a.head<3>() *= s;
  cout << "Translation: " << frt3a.transpose() << endl << endl;

  // calculate cost, should be close to zero
  double cost = 0.0;
  EXPECT_EQ_ABS(cost,0.0,10.0);
  // cameras should be close to their original positions,
  //   adjusted for scale on translations
  for (int i=0; i<4; i++)
    EXPECT_EQ_ABS(sys.nodes[1].qrot.coeffs()[i],frq2.coeffs()[i],0.01);
  for (int i=0; i<4; i++)
    EXPECT_EQ_ABS(sys.nodes[2].qrot.coeffs()[i],frq3.coeffs()[i],0.01);
  for (int i=0; i<3; i++)
    EXPECT_EQ_ABS(frt2a(i),frt2(i),0.05);
  for (int i=0; i<3; i++)
    EXPECT_EQ_ABS(frt3a(i),frt3(i),0.05);

#if 0
  // writing out matrices, 3-node system
  // global system
  sys.useLocalAngles = false;
  nd1.setDr(false);
  nd2.setDr(false);
  nd3.setDr(false);
  sys.setupSys(0.0);
  sys.A.block<6,6>(6,0) = sys.A.block<6,6>(0,6).transpose();
  sys.writeA("A3g.txt");

  // local system
  sys.useLocalAngles = true;
  nd1.setDr(true);
  nd2.setDr(true);
  nd3.setDr(true);
  sys.setupSys(0.0);
  sys.A.block<6,6>(6,0) = sys.A.block<6,6>(0,6).transpose();
  sys.writeA("A3l.txt");
 

  // set up 2-node system
  sys.nodes.clear();
  sys.tracks.clear();

  sys.nodes.push_back(nd1);
  sys.nodes.push_back(nd2);

  // set up projections onto nodes
  ind = 0;
  for(vector<Point,Eigen::aligned_allocator<Point> >::iterator itr = sys.points.begin(); itr!=sys.points.end(); itr++)
    {
      Point pt = *itr;      
      vector<Proj,Eigen::aligned_allocator<Proj> > prjs;	// new point track
      Proj prj;
      prj.isValid = true;
      prj.pti = ind;
      prj.kpi = ind;
      Vector2d ipt;

      n2.setRandom();
      nd1.project2im(ipt,pt);	// set up projection measurement
      prj.ndi = 0;		// nd1 index
      prj.kp = ipt + n2*inoise;
      prjs.push_back(prj);

      n2.setRandom();
      nd2.project2im(ipt,pt);	// set up projection measurement
      prj.ndi = 1;		// nd2 index
      prj.kp = ipt + n2*inoise;
      prjs.push_back(prj);

      sys.tracks.push_back(prjs);
      ind++;
    }

  sys.doSBA(3);

  // writing out matrices, 2-node system
  // global system
  sys.useLocalAngles = false;
  nd1.setDr(false);
  nd2.setDr(false);
  sys.setupSys(0.0);
  sys.writeA("A2g.txt");

  // local system
  sys.useLocalAngles = true;
  nd1.setDr(true);
  nd2.setDr(true);
  sys.setupSys(0.0);
  sys.writeA("A2l.txt");
#endif

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
