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

//
// tolerance for floating-point tests
//

double tol = 1e-5;
#define EXPECT_EQ_TOL(a,b) { if (fabs((a)-(b)) > max(fabs(a),fabs(b))*tol) EXPECT_DOUBLE_EQ(a,b); }
#define EXPECT_EQ_ABS(a,b,t) { if (fabs((a)-(b)) > (t)) EXPECT_DOUBLE_EQ(a,b); }


static Eigen::Matrix<double,6,6> n2prec, n2vprec, n2aprec, n2bprec, diagprec;


void initPrecs()
{
  // set up some precision matrices
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

  n2aprec << 229528.3846633453, 886.7480854882738, -10039.08940223746, 62445.98594207098, 2715273.460194867, 106542.6230004076,
    886.7480854885912, 319242.7032811134, -6397.916315207351, -3608430.146373766, -49269.13482550202, 582748.417531022,
    -10039.08940223649, -6397.916315208951, 565.7603057193751, 69152.18264815415, -117569.9760459389, -16259.89068069827,
    62445.98594206382, -3608430.146373736, 69152.1826481162, 47244836.25653829, 1303537.745687656, -9808843.224988466,
    2715273.46019485, -49269.13482549335, -117569.9760459207, 1303537.745687651, 35830355.245529, 709155.852370202,
    106542.623000413, 582748.4175310251, -16259.89068069991, -9808843.224988459, 709155.8523703497, 48304469.04982638;

  n2bprec << 148324.039595044, 222.4623044457281, -19531.70697504873, -10192.06466578297, 1631677.485087357, 60190.82294241861,
    222.4623044456828, 200041.4398061978, -4054.812572933995, -2258670.079144401, 29578.86052762273, 799843.0721628161,
    -19531.70697504886, -4054.812572933865, 2652.99484259674, 46794.05582115334, -215409.6450292048, -24019.87801347017,
    -10192.06466578462, -2258670.079144401, 46794.05582115659, 28945336.2353294, -434508.6610355716, -12934377.41525949,
    1631677.485087361, 29578.86052762576, -215409.6450292043, -434508.6610355551, 20018126.98420228, 1153711.950184977,
    60190.82294241752, 799843.0721628153, -24019.8780134693, -12934377.41525948, 1153711.950184968, 22955884.81085673;


#if 0
  // this has zeros for rot-trans interaction
  n2prec << 260312.1329351594, 17615.81091248868, -11716.3738046826, 0.0, 0.0, 0.0,
    17615.81091248783, 369156.349498884, -8122.584888439054, 0.0, 0.0, 0.0,
    -11716.3738046842, -8122.58488844048, 673.3469031685361, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 52493931.52684124, -4078689.933502881, -9475682.025736494,
    0.0, 0.0, 0.0, -4078689.933502988, 39416288.19312727, 3894322.443643413,
    0.0, 0.0, 0.0, -9475682.025736755, 3894322.443643621, 50690679.29036696;

  n2vprec << 624875.2423563644,-8.596260869004408e-11,10576.14746839753,0,0,0,
    -1.045228848835824e-10,-2.955857780762017e-10,9.820269042393193e-10,0,0,0,
    10576.14746839765,7.860307960072532e-10,224224.9112157905,0,0,0,
    0,0,0, 7256072.962556601,-1242408.349188809,197719.0360238712,
    0,0,0, -1242408.349188721,214456943.0273151,11161674.13523376,
    0,0,0,  197719.0360238167,11161674.13523367, 19698666.98402661;

  n2aprec << 229528.3846633453, 886.7480854882738, -10039.08940223746, 0,0,0,
    886.7480854885912, 319242.7032811134, -6397.916315207351,  0,0,0,
    -10039.08940223649, -6397.916315208951, 565.7603057193751,  0,0,0,
    0,0,0, 47244836.25653829, 1303537.745687656, -9808843.224988466,
    0,0,0, 1303537.745687651, 35830355.245529, 709155.852370202,
    0,0,0, -9808843.224988459, 709155.8523703497, 48304469.04982638;

  n2bprec << 148324.039595044, 222.4623044457281, -19531.70697504873, 0,0,0,
    222.4623044456828, 200041.4398061978, -4054.812572933995, 0,0,0,
    -19531.70697504886, -4054.812572933865, 2652.99484259674, 0,0,0,
    0,0,0, 28945336.2353294, -434508.6610355716, -12934377.41525949,
    0,0,0, -434508.6610355551, 20018126.98420228, 1153711.950184977,
    0,0,0, -12934377.41525948, 1153711.950184968, 22955884.81085673;
#endif

#if 1
  n2prec  *= 1.0/100000.0;
  n2vprec *= 1.0/100000.0;
  n2aprec *= 1.0/100000.0;
  n2bprec *= 1.0/100000.0;
#endif

  diagprec.setIdentity();
  diagprec = diagprec*(1000);  
  diagprec.diagonal().head(3) *= .0001;
}


#if 1
// test the ConScale jacobian functions
TEST(TestJacobians, TestJacScale)
{
  Quaternion<double> frq1(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  // Quaternion<double> frq1(AngleAxisd(0*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt1(0.2, -0.4, 1.0, 1.0); // frame position in the world
  Quaternion<double> frq2(AngleAxisd(-10*M_PI/180,Vector3d(.4,.3,.1).normalized())); // frame rotation in the world
  // Quaternion<double> frq1(AngleAxisd(0*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt2(-0.2, 0.2, 2.0, 1.0); // frame position in the world

  // difference of rotations
  Quaternion<double> mqp = (frq1.inverse() * frq2).inverse();
  // squared norm of distance
  double ks = (frt2-frt1).squaredNorm();

  Node::initDr();

  // set up frame transform
  Node nd1;
  nd1.qrot = frq1;
  nd1.trans = frt1;
  nd1.setTransform();		// set up world2node transform
  nd1.setDr(true);              // local angles
  //  cout << nd1.dRdx << endl;

  Node nd2;
  nd2.qrot = frq2.coeffs();
  nd2.trans = frt2;
  nd2.setTransform();		// set up world2node transform
  nd2.setDr(true);              // local angles

  std::vector<Node, Eigen::aligned_allocator<Node> > nodes;
  nodes.push_back(nd1);
  nodes.push_back(nd2);

  // compare against numeric Jacobians
  double dq = 1e-9;

  //   this tests the setJacobians method of the ConScale class
  ConScale con;
  con.nd0 = 0;
  con.nd1 = 1;
  con.ks  = ks;
  con.setJacobians(nodes);      // full Jacobian of 2 frames

  Node ndd;

  // compare distance, varying translation params
  {
    Vector4d ti(dq,0,0,0);      // incremental x change
    ndd.qrot = frq1;
    ndd.trans = frt1+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (nd2.trans - ndd.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J0(0));
  }

  {
    Vector4d ti(0,dq,0,0);      // incremental x change
    ndd.qrot = frq1;
    ndd.trans = frt1+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (nd2.trans - ndd.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J0(1));
  }

  {
    Vector4d ti(0,0,dq,0);      // incremental x change
    ndd.qrot = frq1;
    ndd.trans = frt1+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (nd2.trans - ndd.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J0(2));
  }

  {
    Vector4d ti(dq,0,0,0);      // incremental x change
    ndd.qrot = frq2;
    ndd.trans = frt2+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (ndd.trans - nd1.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J1(0));
  }

  {
    Vector4d ti(0,dq,0,0);      // incremental x change
    ndd.qrot = frq2;
    ndd.trans = frt2+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (ndd.trans - nd1.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J1(1));
  }

  {
    Vector4d ti(0,0,dq,0);      // incremental x change
    ndd.qrot = frq2;
    ndd.trans = frt2+ti;
    ndd.setTransform();		// set up world2node transform
  
    double pc1 = (nd2.trans - nd1.trans).squaredNorm() - ks;
    double pc2 = (ndd.trans - nd1.trans).squaredNorm() - ks;
    double pc = pc2 - pc1;
    pc = pc/dq;

    EXPECT_EQ_TOL(pc,con.J1(2));
  }
}


// test ConScale in a simple setup
TEST(TestMono, TestMonoLocalScale)
{
  SysSPA spa;
  Node::initDr();               // set up fixed jacobians

  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;
  double kfang = 5.0;
  double kfrad = kfang*M_PI/180.0;

  spa.nFixed = 3;               // three fixed frames
  spa_spiral_setup(spa, true, cps,
#if 1
                   n2prec, n2vprec, n2aprec, n2bprec,  // rank-deficient
#else
                   diagprec, diagprec, diagprec, diagprec,
#endif
                   kfang, M_PI/2.0-3*kfrad, 220*kfang/360.0, // angle per node, init angle, total nodes
                   0.02, 2.0, 0.01, 0.0, 0.0); // node noise (m,deg), scale noise (increment)

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

#if 1
  // write out pose results and originals
  cout << "[SPAsys] Writing pose file" << endl;  
  ofstream ofs3("P400.init.txt");
  for (int i=0; i<(int)cps.size(); i++)
    {
      Vector3d tpn = spa.nodes[i].trans.head(3);
      ofs3 << tpn.transpose() << endl;
    }  
  ofs3.close();
#endif


  // add in distance constraint
  // works with either n0 -> ni or ni -> ni+1 constraints
#if 0
  ConScale con;
  con.w = 0.1;                // weight
  for (int i=0; i<(int)cps.size()-3; i++)
    {
      spa.scales.push_back(1.0);
      con.nd0 = i;              // first node
      con.nd1 = i+1;            // second node
      con.sv  = i;              // scale index
      con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);

      con.nd0 = i+1;            // first node
      con.nd1 = i+2;            // second node
      con.sv  = i;              // scale index
      con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);

      con.nd0 = i+2;            // first node
      con.nd1 = i+3;            // second node
      con.sv  = i;              // scale index
      con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);

      con.nd0 = i;              // first node
      con.nd1 = i+3;            // second node
      con.sv  = i;              // scale index
      con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);


#if 0                           // doesn't seem to help...
      if (i>2)
        {
          con.nd0 = i-3;              // first node
          con.nd1 = i+1;            // second node
          con.sv  = 0;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);
        }
#endif

#if 0
      if (i>72)
        {
          con.nd1 = i;
          con.nd0 = i-72;
          con.sv  = 0;
          con.ks = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);
        }
#endif

    }
#endif


  // add in cross-link distance constraint
  // not much effect here...
#if 0
  ConScale con;
  con.w = 0.1;                // weight
  for (int i=72; i<(int)cps.size()-3; i++)
    {
      spa.scales.push_back(1.0);
      con.nd0 = i;              // first node
      con.nd1 = i-2;            // second node
      con.sv  = i-72;           // scale index
      con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);

      con.nd0 = i-72;
      con.nd1 = i;
      con.sv  = i-72;
      con.ks = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
      spa.scons.push_back(con);
    }
#endif



  // this adds in a global constraint connecting the two sides
#if 1
  {
    ConP2 con;
    con.ndr = 0;
    con.nd1 = 36;
    Node nd0 = spa.nodes[con.ndr];
    Node nd1 = spa.nodes[con.nd1];
    Vector4d trans;
    trans.head(3) = cps[con.nd1].head(3);
    trans(3) = 1.0;

    con.prec = 1000*diagprec;

    con.tmean = nd0.w2n * trans; // translation offset
    Quaternion<double> q0,q1,qm;
    q0 = nd0.qrot;
    q1 = nd1.qrot;
    con.qpmean = (q0.inverse()*q1).inverse();

    spa.p2cons.push_back(con);
  }
#endif

  {
  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      Vector3d err = tp-tpn;
      sqerr += err.squaredNorm();

      Quaternion<double> qr;
      qr.vec() = cp.block<3,1>(3,0);
      qr.w() = sqrt(1.0 - qr.vec().squaredNorm());
      Quaternion<double> qn;
      qn = spa.nodes[i].qrot;
      double da = qr.angularDistance(qn);
      asqerr += da;

    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));
  }

  long long t0, t1;
  t0 = utime();
  int niters = spa.doSPA(10);
  t1 = utime();
  printf("[TestSPA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)niters);
  printf("[TestSPA] Accepted iterations: %d\n", niters);
  cout << "[SPA Spiral] Final cost is " << spa.calcCost() << endl;

#if 0
  // write out A matrix
  spa.setupSys(0.0);
  cout << "[SPAsys] Writing file" << endl;
  spa.writeSparseA("A400mono.txt");
#endif

#if 1
  // write out pose results and originals
  cout << "[SPAsys] Writing pose file" << endl;  
  ofstream ofs1("P400.ground.txt");
  ofstream ofs2("P400.optim.txt");
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      ofs1 << tp.transpose() << endl;
      ofs2 << tpn.transpose() << endl;
    }  
  ofs1.close();
  ofs2.close();
#endif
  

  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      Vector3d err = tp-tpn;
      sqerr += err.squaredNorm();

      Quaternion<double> qr;
      qr.vec() = cp.block<3,1>(3,0);
      qr.w() = sqrt(1.0 - qr.vec().squaredNorm());
      Quaternion<double> qn;
      qn = spa.nodes[i].qrot;
      double da = qr.angularDistance(qn);
      asqerr += da;

    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));

  EXPECT_EQ_ABS(sqerr,0.0,500e-3); // should be within 500mm
  EXPECT_EQ_ABS(asqerr,0.0,0.3); // should be within .3 deg
}
#endif

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  initPrecs();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
