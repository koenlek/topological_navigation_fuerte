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
#include "bpcg/bpcg.h"

// Bring in gtest
#include <gtest/gtest.h>

using namespace Eigen;
using namespace sba;

#include <iostream>
#include <fstream>
using namespace std;

// test the matrix-vector multiplication system
TEST(bpcg, mMV)
{
  // set up dense system
  // each 6x6 block is symmetric and positive definite
  // the whole matrix is symmetric
  // is it positive definite?  ...nope
  Matrix<double,24,24> M, K, R;	// dense system matrix
  M.setRandom();
  // make each 6x6 block symmetric, as well as the whole matrix
  for (int i=0; i<4; i++)
    for (int j=i; j<4; j++)
      {
	M.block(i*6,j*6,6,6) = M.block(i*6,j*6,6,6).transpose()*M.block(i*6,j*6,6,6);
	M.block(j*6,i*6,6,6) = M.block(i*6,j*6,6,6);
      }
  M.block(12,0,6,6).setZero();	// zero out some entries
  M.block(0,12,6,6).setZero();

  VectorXd v(24), vm(24), vmm(24); // full system vectors
  v.setRandom();

  vm = M*v;

  // set up sparse system
  vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > diag;
  vector< map<int,Matrix<double,6,6>, less<int>, aligned_allocator<Matrix<double,6,6> > > > cols;
  VectorXd vin(24), vout(24), x0(24);

  // create vectors and matrix
  x0.setZero();
  diag.resize(4);
  cols.resize(4);
  for (int i=0; i<4; i++)
    {
      int ii = 6*i;
      diag[i] = M.block(ii,ii,6,6);

      map<int,Matrix<double,6,6>, less<int>, 
	aligned_allocator<Matrix<double,6,6> > > &col = cols[i];
      for (int j=i+1; j<4; j++)
	if (!(j==2 && i==0))	// these blocks are not zero
	  col.insert(pair<int,Matrix<double,6,6> >(j,M.block(j*6,ii,6,6)));
    }

  // multiply
  mMV(diag,cols,v,vout);

  cout << v.transpose() << endl;
  cout << vout.transpose() << endl;
  cout << vm.transpose() << endl;

  // compare outputs
  for (int i=0; i<vm.rows(); i++)
    EXPECT_FLOAT_EQ(vm[i],vout[i]);

  vin = vout;

#if 1
  // simple case
  //  diag.resize(1);
  cols.resize(1);
  //  x0.setZero(6);
  //  vin.setZero(6);
  //  vin = vout.start(6);
#endif

  // set up simple test
  MatrixXd MM;
  MM.setZero(12,12);
  MM << 727624.1239206992,243583.6638335874,136406.3699845557,-649049.2057823476,3177504.726639398,-875433.9245907085,-523918.6146375744,-247000.0715695264,-185935.0054013509,1700939.169375825,-1918392.643847066,-694586.1808429885,
    243583.6638335872,257241.5854867174,97918.95660890511,-801347.9323137584,1233703.865879556,-446717.1185187973,-234322.1521862475,-138738.9112753089,-43582.59853617808,800916.1647884385,-912287.2295583673,-294625.2334960104,
    136406.3699845563,97918.95660890498,536410.0970080961,-20024.19716237042,875418.3206046737,-74881.80657696929,-152138.6532112983,-111606.5312919594,-507967.6608696034,1054577.587722445,-360152.6926145303,-275341.3454739356,
    -649049.2057823451,-801347.9323137584,-20024.19716237064,3735469.787162997,-3622617.915957548,1026576.219219045,713673.2451608701,369642.9319339712,-140694.2692220075,-2816161.328356379,3034044.144042888,1157578.584878814,
    3177504.7266394,1233703.865879555,875418.3206046739,-3622617.915957554,15091189.80671031,-3841243.239395392,-2418631.899173797,-1182633.33794011,-1014829.937686714,8629983.77851025,-9282477.133261677,-3223356.600356609,
    -875433.9245907082,-446717.1185187968,-74881.80657696963,1026576.219219047,-3841243.239395393,3529022.2517114,515813.4290909402,260815.0170004668,74615.43621058742,-1447359.804337021,2269479.344498967,-794218.0087071136,
    -523918.6146375744,-234322.1521862475,-152138.6532112983,713673.2451608701,-2418631.899173797,515813.4290909402,731494.3618955865,236381.8229705717,210817.9916922446,-1831830.813800446,2642745.758829073,1183523.336234943,
    -247000.0715695264,-138738.9112753089,-111606.5312919594,369642.9319339712,-1182633.33794011,260815.0170004668,236381.8229705716,235585.179073937,66896.99672782572,-1192988.98662512,845299.8075046183,423528.8050741359,
    -185935.0054013509,-43582.59853617808,-507967.6608696034,-140694.2692220075,-1014829.937686714,74615.43621058742,210817.9916922438,66896.99672782577,542230.0471322589,-991900.0725815309,550697.285341059,358713.944182454,
    1700939.169375825,800916.1647884385,1054577.587722445,-2816161.328356379,8629983.77851025,-1447359.804337021,-1831830.813800443,-1192988.986625121,-991900.0725815326,8508820.768793903,-6580932.594742717,-3375523.795370819,
    -1918392.643847066,-912287.2295583673,-360152.6926145303,3034044.144042888,-9282477.133261677,2269479.344498967,2642745.758829073,845299.8075046184,550697.2853410626,-6580932.594742729,10493795.54564034,3764746.417176266,
    -694586.1808429885,-294625.2334960104,-275341.3454739356,1157578.584878814,-3223356.600356609,-794218.0087071136,1183523.336234942,423528.8050741353,358713.9441824536,-3375523.795370816,3764746.417176265,4104760.618614043;

  vin = vin.start(12);
  vout = MM * vin;
  vout << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  x0.setZero(12);

  // try the bpcg algorithm
  bpcg_jacobi_dense(MM.cols()*2,1e-10,MM,x0,vout);
  cout << endl;


  diag.resize(2);
  cols.resize(2);
  for (int i=0; i<2; i++)
    {
      int ii = 6*i;
      diag[i] = MM.block(ii,ii,6,6);

      map<int,Matrix<double,6,6>, less<int>, 
	aligned_allocator<Matrix<double,6,6> > > &col = cols[i];
      col.clear();
      for (int j=i+1; j<2; j++)
	col.insert(pair<int,Matrix<double,6,6> >(j,MM.block(j*6,ii,6,6)));
    }

  // try the bpcg algorithm
  x0.setZero(12);
  bpcg_jacobi(diag.size()*12,1e-10,diag,cols,x0,vout);


  mMV(diag,cols,vin,vout);
  x0 = MM*vin;

  // compare outputs
  for (int i=0; i<x0.rows(); i++)
    EXPECT_FLOAT_EQ(x0[i],vout[i]);

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
