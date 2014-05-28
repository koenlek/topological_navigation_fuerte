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

// test fixture for Sparse Pose Adjustment

#include <iostream>
using namespace std;

// Bring in my package's API, which is what I'm testing
#include "sba/sba_setup.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace Eigen;
using namespace sba;
using namespace frame_common;

//
// tolerance for floating-point tests
//

double tol = 1e-5;
#define EXPECT_EQ_TOL(a,b) { if (fabs((a)-(b)) > max(fabs(a),fabs(b))*tol) EXPECT_DOUBLE_EQ(a,b); }
#define EXPECT_EQ_ABS(a,b,t) { if (fabs((a)-(b)) > (t)) EXPECT_DOUBLE_EQ(a,b); }


static Eigen::Matrix<double,3,3> n2prec, n2vprec, diagprec;


void initPrecs()
{
  // set up some precision matrices
#if 0
  n2prec << 260312.1329351594, 17615.81091248868, -11716.3738046826,
           -260221.3577238563, 3028947.570775249, 284048.6838048229,
    17615.81091248783, 369156.349498884, -8122.584888439054,
    -4130281.103526653, 265383.1196958761, 523737.7444220608,
    -11716.3738046842, -8122.58488844048, 673.3469031685361,
    93635.22686723019, -137533.0434459766, -22834.5012408561,
    -260221.3577238639, -4130281.103526646, 93635.22686720481,
    52493931.52684124, -4078689.933502881, -9475682.025736494,
    3028947.570775286, 265383.119695912, -137533.0434459558,
    -4078689.933502988, 39416288.19312727, 3894322.443643413,
    284048.6838048277, 523737.7444220638, -22834.50124085596,
    -9475682.025736755, 3894322.443643621, 50690679.29036696;

  n2vprec << 624875.2423563644,-8.596260869004408e-11,10576.14746839753,
    -65704.86829639168,10646337.23355757,646569.8439109828,
    -1.045228848835824e-10,-2.955857780762017e-10,9.820269042393193e-10,
    6.912159733474255e-09,-3.751665644813329e-09,-3.511559043545276e-08,
    10576.14746839765,7.860307960072532e-10,224224.9112157905,
    -233966.3120641535,77714.35666432518,65704.86829639639,
    -65704.86829639156,8.021743269637227e-09,-233966.312064158,
    7256072.962556601,-1242408.349188809,197719.0360238712,
    10646337.23355758,-6.682398634438869e-09,77714.35666432098,
    -1242408.349188721,214456943.0273151,11161674.13523376,
    646569.8439109783,-3.356490196892992e-08,65704.86829639817,
    197719.0360238167,11161674.13523367, 19698666.98402661;
#endif

  diagprec.setIdentity();
  //  diagprec = diagprec*(1000);  
  //  diagprec.diagonal().head(2) *= .0001;
}



// test the ConP2 jacobian functions
TEST(TestJacobians, TestJac2dP2)
{
  double a1 = 10*M_PI/180;
  Vector3d frt1(0.2, -0.4, 1.0); // frame position in the world
  double a2 = -10*M_PI/180;
  Vector3d frt2(-0.2, 0.2, 1.0); // frame position in the world

  // difference of rotations
  double am = a2 - a1;

  // set up frame transform
  Node2d nd1;
  nd1.arot = a1;
  nd1.trans = frt1;
  nd1.setTransform();		// set up world2node transform
  nd1.setDr();                  // local angles
  //  cout << nd1.dRdx << endl;

  Node2d nd2;
  nd2.arot = a2;
  nd2.trans = frt2;
  nd2.setTransform();		// set up world2node transform
  nd2.setDr();                  // local angles

  std::vector<Node2d, Eigen::aligned_allocator<Node2d> > nodes;
  nodes.push_back(nd1);
  nodes.push_back(nd2);

  // compare against numeric Jacobians
  double dq = 1e-9;

  // set up a projected point, test Jacobian of pose constraint
  //   this tests the setJacobians method of the ConP2 class, for local angles
  Con2dP2 con;
  con.ndr = 0;
  con.nd1 = 1;
  con.amean = am;
  con.setJacobians(nodes);      // full Jacobian of 2 frames

  // compare projections, varying angle params
  Node2d ndd;
  {
    double ai = dq;
    ndd.arot  = nd1.arot + ai;
    ndd.trans = frt1;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = ndd.w2n * nd2.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J0(i,2));
  }

  // J1
  {
    double ai = dq;
    ndd.arot  = nd2.arot + ai;
    ndd.trans = frt2;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = nd1.w2n * ndd.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J1(i,2));
  }


  // compare projections, varying translation params
  {
    Vector3d ti(dq,0,0);        // incremental x change
    ndd.arot = a1;
    ndd.trans = frt1+ti;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = ndd.w2n * nd2.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J0(i,0));
  }

  {
    Vector3d ti(0,dq,0);        // incremental y change
    ndd.arot = a1;
    ndd.trans = frt1+ti;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = ndd.w2n * nd2.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J0(i,1));
  }


  // J1
  {
    Vector3d ti(dq,0,0);        // incremental x change
    ndd.arot = a2;
    ndd.trans = frt2+ti;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = nd1.w2n * ndd.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J1(i,0));
  }

  {
    Vector3d ti(0,dq,0);        // incremental y change
    ndd.arot = a2;
    ndd.trans = frt2+ti;
    ndd.setTransform();		// set up world2node transform
  
    Vector2d pc1 = nd1.w2n * nd2.trans;
    Vector2d pc2 = nd1.w2n * ndd.trans;
    Vector2d pc = pc2 - pc1;
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),con.J1(i,1));
  }


  // compare angles, varying angle param of nd1
  // measurement eq is (a2 - a1) - am
  {
    double ai = dq;
    double ad = (a2-a1)-am;
    double add = (a2-(a1+ai))-am;
    double pc = (add - ad)/dq;

    EXPECT_EQ_TOL(pc,con.J0(2,2));
  }

  // J1
  {
    double ai = dq;
    double ad = (a2-a1)-am;
    double add = ((a2+ai)-a1)-am;
    double pc = (add - ad)/dq;

    EXPECT_EQ_TOL(pc,con.J1(2,2));
  }

}


// test a simple linear SPA
TEST(TestSPA2d, TestSimple2nodes)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;

  spa2d_spiral_setup(spa, cps,
                     diagprec, diagprec, diagprec, diagprec,
                     kfang, M_PI/2.0-.001, 2*kfang/360.0, // angle per node, init angle, total nodes
                     0.0, 0.0, 0.0, 0.02, 1.0); // node noise (m,deg), meas noise (m,deg)

  Node2d &nd = spa.nodes[1];
  double qnoise = M_PI*5.0/180.0; // convert to radians

  nd.arot += qnoise;
  nd.normArot();

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int niters = spa.doSPA(4,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  EXPECT_EQ_ABS(spa.calcCost(), 0.0, 1.0e-12);
}

TEST(TestSPA2d, TestSimple3nodes)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;

  spa2d_spiral_setup(spa, cps,
                   diagprec, diagprec,diagprec, diagprec,
                   kfang, -M_PI/2, 3*kfang/360.0, // angle per node, init angle, total nodes
                   0.0, 0.0, 0.0, 0.0, 0.0); // node noise (m,deg), meas noise (m,deg)

  Node2d &nd0 = spa.nodes[1];
  Node2d &nd1 = spa.nodes[2];
  double qnoise = M_PI*5.0/180.0; // convert to radians

  nd0.arot += qnoise;
  nd0.normArot();

  nd1.arot -= qnoise;
  nd1.normArot();

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int niters = spa.doSPA(4,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  EXPECT_EQ_ABS(spa.calcCost(), 0.0, 1.0e-12);

}

TEST(TestSPA2d, TestMarthi3nodes)
{
  SysSPA2d spa;

  int n0 = spa.addNode(Vector3d(0,0,0),0);
  int n1 = spa.addNode(Vector3d(2,5,0),1);
  int n2 = spa.addNode(Vector3d(9,1,0),2);
  printf("Nodes: %d %d %d\n", n0, n1, n2);

  Matrix3d prec;
  prec.setIdentity();
  spa.addConstraint(n0,n1,Vector3d(1,0,0),prec);
  spa.addConstraint(n0,n2,Vector3d(0.5,1,0),prec);
  spa.addConstraint(n2,n1,Vector3d(0.5,-1,0),prec);

  Node2d &nd0 = spa.nodes[1];
  Node2d &nd1 = spa.nodes[2];
  double qnoise = M_PI*5.0/180.0; // convert to radians

  nd0.arot += qnoise;
  nd0.normArot();

  nd1.arot -= qnoise;
  nd1.normArot();

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int niters = spa.doSPA(4,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  cout << "Node 1 pose: " << nd0.trans.transpose() << endl;
  cout << "Node 2 pose: " << nd1.trans.transpose() << endl;

  EXPECT_EQ_ABS(spa.calcCost(), 0.0, 1.0e-12);

}



// these should all be redone with displacement noise
TEST(TestSPA2d, TestSimple10_i90)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;
  double kfrad = kfang*M_PI/180.0;

  spa2d_spiral_setup(spa, cps,
                   diagprec, diagprec,diagprec, diagprec,
                   kfang, M_PI/2-3*kfrad, 20*kfang/360.0, // angle per node, init angle, total nodes
                   0.02, 1.0, 0.02, 0.02, 1.0); // incremental node noise (m,deg), scale noise (%), displacement (m,deg)

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int niters = spa.doSPA(10,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  // test results
  double sqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,3,1> &cp = cps[i]; // old camera pose
      Vector2d tp = cp.head(2);
      Vector2d tpn = spa.nodes[i].trans.head(2);
      //      printf("\n[TestSPA2d] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA2d] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);
      Vector2d err = tp-tpn;
      sqerr += err.squaredNorm();
    }
  
  sqerr = sqerr / (double)(cps.size());
  printf("\n[TestSPA2d] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-2); // should be within 15mm


}


TEST(TestSPA2d, TestSimple10_i0)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;
  double kfrad = kfang*M_PI/180.0;

  spa2d_spiral_setup(spa, cps,
                   diagprec, diagprec,diagprec, diagprec,
                   kfang, 0-kfrad, 20*kfang/360.0, // angle per node, init angle, total nodes
                   0.02, 1.0, 0.02, 0.02, 1.0); // incremental node noise (m,deg), scale noise (%), displacement (m,deg)

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int niters = spa.doSPA(10,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  // test results
  double sqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,3,1> &cp = cps[i]; // old camera pose
      Vector2d tp = cp.head(2);
      Vector2d tpn = spa.nodes[i].trans.head(2);
      //      printf("\n[TestSPA2d] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA2d] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);
      Vector2d err = tp-tpn;
      sqerr += err.squaredNorm();
    }
  
  sqerr = sqerr / (double)(cps.size());
  printf("\n[TestSPA2d] RMSE is %0.3f m\n", sqrt(sqerr));

  EXPECT_EQ_ABS(sqerr,0.0,15e-2); // should be within 15mm
}


TEST(TestSPA2d, TestSimple400nodesSparse)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;

  spa2d_spiral_setup(spa, cps,
                   diagprec, diagprec,diagprec, diagprec,
                   kfang, -M_PI/2, 2500*kfang/360.0, // angle per node, init angle, total nodes
                   0.02, 1.0, 0.0, 0.02, 2.0); // node noise (m,deg), meas noise (m,deg)

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  //  spa.setupSys(0.0);
  //  spa.writeSparseA("A400.sptxt");

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int doiters = 10;
  int niters = spa.doSPA(doiters,1.0e-4,1);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)doiters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,3,1> &cp = cps[i]; // old camera pose
      Vector2d tp = cp.head(2);
      Vector2d tpn = spa.nodes[i].trans.head(2);
      //      printf("\n[TestSPA2d] Cam %d orig: %0.2f %0.2f\n", i, tp[0], tp[1]);
      //      printf("[TestSPA2d] Cam %d new:  %0.2f %0.2f\n", i, tpn[0], tpn[1]);
      Vector2d err = tp-tpn;
      sqerr += err.squaredNorm();

      double da = spa.nodes[i].arot - cp(2);
      if (da > M_PI) da -= 2.0*M_PI;
      if (da < -M_PI) da += 2.0*M_PI;
      asqerr += da*da;
    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA2d] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));

  EXPECT_EQ_ABS(sqerr,0.0,1.5); // should be within 1.5m, depending on the test
  EXPECT_EQ_ABS(asqerr,0.0,15e-2); // should be within .15 deg

#if 0
  char fname[256];
  sprintf(fname,"spiral-%d.txt",nnodes);
  spa.writeSparseA(fname,true);

  spa.doSPA(doiters,1.0e-2,0);
#endif

}


TEST(TestSPA2d, TestSimple400nodes)
{
  SysSPA2d spa;

  vector<Matrix<double,3,1>,Eigen::aligned_allocator<Matrix<double,3,1> > > cps;
  double kfang = 5.0;

  spa2d_spiral_setup(spa, cps,
                   diagprec, diagprec,diagprec, diagprec,
                   kfang, -M_PI/2, 400*kfang/360.0, // angle per node, init angle, total nodes
                   0.02, 1.0, 0.0, 0.02, 2.0); // node noise (m,deg), meas noise (m,deg)

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

  //  spa.setupSys(0.0);
  //  spa.writeSparseA("A400.sptxt");

  long long t0, t1;
  t0 = utime();
  spa.nFixed = 1;               // one fixed frame
  int doiters = 10;
  int niters = spa.doSPA(doiters,1.0e-4,0);
  t1 = utime();
  printf("[TestSPA2d] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)doiters);
  printf("[TestSPA2d] Accepted iterations: %d\n", niters);

  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,3,1> &cp = cps[i]; // old camera pose
      Vector2d tp = cp.head(2);
      Vector2d tpn = spa.nodes[i].trans.head(2);
      //      printf("\n[TestSPA2d] Cam %d orig: %0.2f %0.2f\n", i, tp[0], tp[1]);
      //      printf("[TestSPA2d] Cam %d new:  %0.2f %0.2f\n", i, tpn[0], tpn[1]);
      Vector2d err = tp-tpn;
      sqerr += err.squaredNorm();

      double da = spa.nodes[i].arot - cp(2);
      if (da > M_PI) da -= 2.0*M_PI;
      if (da < -M_PI) da += 2.0*M_PI;
      asqerr += da*da;
    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA2d] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));

  EXPECT_EQ_ABS(sqerr,0.0,1.5); // should be within 1.5m, depending on the test
  EXPECT_EQ_ABS(asqerr,0.0,15e-2); // should be within .15 deg

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  initPrecs();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

