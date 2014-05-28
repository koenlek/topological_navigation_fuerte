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

#include <iostream>
using namespace std;

// Bring in my package's API, which is what I'm testing
#include "sba/sba.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace Eigen;
using namespace sba;
using namespace frame_common;

//
// tolerance for floating-point tests
//

double tol = 1e-6;
#define EXPECT_EQ_TOL(a,b) { if (fabs((a)-(b)) > max(fabs(a),fabs(b))*tol) EXPECT_DOUBLE_EQ(a,b); }


// test the transform functions
TEST(TestTransform, transformW2C)
{
//  <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
  Quaternion<double> frq1(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt1(0.2, -0.4, 1.0, 1.0); // frame position in the world
  Vector4d pt1(0.5, 0.2, 3.0, 1.0); // homogenous coordinates
  Matrix<double, 3, 4> pmat;	// projection matrix
  transformW2F(pmat, frt1, frq1); // create projection transform
  Vector3d pt2 = pmat*pt1;

#if 0
  cout << frt1 << endl << endl;
  cout << pmat << endl << endl;
  cout << pmat*pt1 << endl << endl; // do the transform
  cout << pt2[0]/pt2[2] << "  " << pt2[1]/pt2[2] << endl;
#endif

  EXPECT_EQ_TOL(0.096004, pt2[0]/pt2[2]);
  EXPECT_EQ_TOL(0.351973, pt2[1]/pt2[2]);
}


// test the projection functions
TEST(TestProjection, projectW2C)
{
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  //  cout << fr1.iproj << endl;

  Quaternion<double> frq1(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt1(0.2, -0.4, 1.0, 1.0); // frame position in the world
  Vector4d pt1(0.5, 0.2, 3.0, 1.0); // homogeneous coordinates

  // set up frame transform
  Node nd1;
  nd1.qrot = frq1.coeffs();
  nd1.trans = frt1;
  nd1.setTransform();		// set up world2node transform
  nd1.setKcam(cpars);		// set up node2image projection

  Vector3d pt2 = nd1.w2n*pt1;
  Vector3d pt3 = nd1.w2i*pt1;


  EXPECT_EQ_TOL(0.096004, pt2[0]/pt2[2]);
  EXPECT_EQ_TOL(0.351973, pt2[1]/pt2[2]);

  EXPECT_EQ_TOL(348.80121, pt3[0]/pt3[2]);
  EXPECT_EQ_TOL(345.59192, pt3[1]/pt3[2]);
}


// test the jacobian functions
TEST(TestJacobians, TestJacGlobalAngles)
{
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  //  cout << fr1.iproj << endl;

  Quaternion<double> frq1(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  // Quaternion<double> frq1(AngleAxisd(0*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt1(0.2, -0.4, 1.0, 1.0); // frame position in the world
  //  Vector4d frt1(0.0, 0.0, 1.0, 1.0); // frame position in the world
  Vector4d pt1(0.5, 0.2, 3.0, 1.0); // homogeneous coordinates

  // set up frame transform
  Node nd1;
  nd1.qrot = frq1.coeffs();
  nd1.trans = frt1;
  nd1.setTransform();		// set up world2node transform
  nd1.setKcam(cpars);		// set up node2image projection

  // Rotational derivatives
  nd1.setDr(false);             // global angles

  // compare against numeric Jacobians
  double dq = 1e-8;
  double s = Proj::qScale;

  // here we test the rotation matrix derivative,
  //  calculated in the Node setDr() method
  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(dq,0,0,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    Quaternion<double> frq2(cs2);
    Matrix<double,3,3> Rp = frq2.toRotationMatrix().transpose();
    Matrix<double,3,3> R =  frq1.toRotationMatrix().transpose();

    // check projection against numeric derivative
    Vector3d pc1 = R  * pt1.head(3);
    Vector3d pc2 = Rp * pt1.head(3);
    Vector3d pc  = (pc2 - pc1)/dq;
    Vector3d pcd = nd1.dRdx * pt1.head(3); // analytic derivative

    //    cout << pc1 << endl << endl;
    //    cout << pc  << endl << endl;
    //    cout << pcd << endl << endl;

    for (int i=0; i<3; i++)
      EXPECT_EQ_TOL(pc(i),pcd(i));
  }

  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(0,dq,0,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    Quaternion<double> frq2(cs2);
    Matrix<double,3,3> Rp = frq2.toRotationMatrix().transpose();
    Matrix<double,3,3> R =  frq1.toRotationMatrix().transpose();

    // check projection against numeric derivative
    Vector3d pc1 = R  * pt1.head(3);
    Vector3d pc2 = Rp * pt1.head(3);
    Vector3d pc  = (pc2 - pc1)/dq;
    Vector3d pcd = nd1.dRdy * pt1.head(3); // analytic derivative

    //    cout << pc1 << endl << endl;
    //    cout << pc  << endl << endl;
    //    cout << pcd << endl << endl;

    for (int i=0; i<3; i++)
      EXPECT_EQ_TOL(pc(i),pcd(i));
  }

  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(0,0,dq,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    Quaternion<double> frq2(cs2);
    Matrix<double,3,3> Rp = frq2.toRotationMatrix().transpose();
    Matrix<double,3,3> R =  frq1.toRotationMatrix().transpose();

    // check projection against numeric derivative
    Vector3d pc1 = R  * pt1.head(3);
    Vector3d pc2 = Rp * pt1.head(3);
    Vector3d pc  = (pc2 - pc1)/dq;
    Vector3d pcd = nd1.dRdz * pt1.head(3); // analytic derivative

    //    cout << pc1 << endl << endl;
    //    cout << pc  << endl << endl;
    //    cout << pcd << endl << endl;

    for (int i=0; i<3; i++)
      EXPECT_EQ_TOL(pc(i),pcd(i));
  }

  // set up a projected point, test Jacobian of projection
  //   this tests the setJacobians method of the Proj class, for global angles
  Proj proj;
  proj.setJacobians(nd1,pt1); // full Jacobian of projected point wrt camera
                                //    and point params, global angles
  // compare projections, varying quaternion params
  Node nd2;
  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(dq,0,0,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,3));
  }

  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(0,dq,0,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,4));
  }

  {
    Vector4d cs2 = frq1.coeffs()+Vector4d(0,0,dq,0);
    cs2(3) = sqrt(1.0 - cs2.head(3).squaredNorm()); // have to normalize w!!!!
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,5));
  }

  // compare projections, varying translation params
  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1 + Vector4d(dq,0,0,0);
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,0));
  }
 
  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1 + Vector4d(0,dq,0,0);
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,1));
  }

  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1 + Vector4d(0,0,dq,0);
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,2));
  }

  // compare projections, varying point params
  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * (pt1 + Vector4d(dq,0,0,0));
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacp(i,0));
  }
 
  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * (pt1 + Vector4d(0,dq,0,0));
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacp(i,1));
  }

  {
    Vector4d cs2 = frq1.coeffs();
    nd2.qrot = cs2;
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * (pt1 + Vector4d(0,0,dq,0));
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacp(i,2));
  }

}


// test the jacobian functions
TEST(TestJacobians, TestJacLocalAngles)
{
  CamParams cpars = {300,300,320,240,0}; // 300 pix focal length

  //  cout << fr1.iproj << endl;

  Quaternion<double> frq1(AngleAxisd(10*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  // Quaternion<double> frq1(AngleAxisd(0*M_PI/180,Vector3d(.2,.3,.4).normalized())); // frame rotation in the world
  Vector4d frt1(0.2, -0.4, 1.0, 1.0); // frame position in the world
  //  Vector4d frt1(0.0, 0.0, 1.0, 1.0); // frame position in the world
  Vector4d pt1(0.5, 0.2, 3.0, 1.0); // homogeneous coordinates

  Node::initDr();               // set up constant derivative matrices

  // set up frame transform
  Node nd1;
  nd1.qrot = frq1.coeffs();
  nd1.trans = frt1;
  nd1.setTransform();		// set up world2node transform
  nd1.setKcam(cpars);		// set up node2image projection

  // Rotational derivatives, local angles
  nd1.setDr(true);              // local angles

  // compare against numeric Jacobians
  double dq = 1e-8;
  double wdq = sqrt(1.0 - dq*dq);
  double s = Proj::qScale;

  // set up a projected point, test Jacobian of projection
  //   this tests the setJacobians method of the Proj class, for local angles
  Proj proj;
  proj.setJacobians(nd1,pt1);   // full Jacobian of projected point wrt camera
                                //    and point params, local angles
  // compare projections, varying local quaternion params
  Node nd2;
  {
    Quaternion<double> frqi(wdq,dq,0,0); // incremental angle change
    frqi = frq1*frqi;           // post-multiply
    
    nd2.qrot = frqi.coeffs();
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,3));
  }

  {
    Quaternion<double> frqi(wdq,0,dq,0); // incremental angle change
    frqi = frq1*frqi;           // post-multiply
    
    nd2.qrot = frqi.coeffs();
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,4));
  }

  {
    Quaternion<double> frqi(wdq,0,0,dq); // incremental angle change
    frqi = frq1*frqi;           // post-multiply
    
    nd2.qrot = frqi.coeffs();
    nd2.trans = frt1;
    nd2.setTransform();		// set up world2node transform
    nd2.setKcam(cpars);		// set up node2image projection
  
    Vector3d pc1 = nd1.w2i * pt1;
    Vector3d pc2 = nd2.w2i * pt1;
    Vector2d pc = pc2.head(2)/pc2(2) - pc1.head(2)/pc1(2);
    pc = s*pc/dq;

    for (int i=0; i<2; i++)
      EXPECT_EQ_TOL(pc(i),proj.jacc(i,5));
  }

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

