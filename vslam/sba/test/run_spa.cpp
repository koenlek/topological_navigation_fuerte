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
// Running reduced pose system
//

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "sba/read_spa.h"
#include "sba/sba.h"
#include <Eigen/Cholesky>

using namespace Eigen;
using namespace std;
using namespace sba;

#include <sys/time.h>

// elapsed time in microseconds
static long long utime()
{
  timeval tv;
  gettimeofday(&tv,NULL);
  long long ts = tv.tv_sec;
  ts *= 1000000;
  ts += tv.tv_usec;
  return ts;
}


//
// add a single node to the graph, in the position given by the VERTEX2 entry in the file
//

void 
addnode(SysSPA &spa, int n, 
	// node translation
	std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ntrans,
	// node rotation
	std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > nqrot,
	// constraint indices
	std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind,
	// constraint local translation 
	std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ctrans,
	// constraint local rotation as quaternion
	std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > cqrot,
	// constraint covariance
	std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > cvar)

{
  Node nd;

  // rotation
  Quaternion<double> frq;
  frq.coeffs() = nqrot[n];
  frq.normalize();
  if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
  nd.qrot = frq.coeffs();

  // translation
  Vector4d v;
  v.head(3) = ntrans[n];
  v(3) = 1.0;
  nd.trans = v;
  nd.setTransform();        // set up world2node transform
  nd.setDr(true);

  // add to system
  spa.nodes.push_back(nd);

  // add in constraints
  for (int i=0; i<(int)ctrans.size(); i++)
    {
      ConP2 con;
      con.ndr = cind[i].x();
      con.nd1 = cind[i].y();

      if ((con.ndr == n && con.nd1 <= n-1) ||
          (con.nd1 == n && con.ndr <= n-1))
        {
	  con.tmean = ctrans[i];
	  Quaternion<double> qr;
	  qr.coeffs() = cqrot[i];
	  qr.normalize();
	  con.qpmean = qr.inverse(); // inverse of the rotation measurement
	  con.prec = cvar[i];       // ??? should this be inverted ???

	  // need a boost for noise-offset system
	  //con.prec.block<3,3>(3,3) *= 10.0;
	  spa.p2cons.push_back(con);
	}
    }
}


//
// first argument is the name of input file.
// files are in Freiburg's VERTEX3/EDGE3 format
// runs SPA
//

int main(int argc, char **argv)
{
  char *fin;

  if (argc < 2)
    {
      cout << "Arguments are:  <input filename> [<number of nodes to use>]" << endl;
      return -1;
    }

  // number of nodes to use
  int nnodes = -1;

  if (argc > 2)
    nnodes = atoi(argv[2]);

  int doiters = 10;
  if (argc > 3)
    doiters = atoi(argv[3]);

  fin = argv[1];

  // node translation
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ntrans;
  // node rotation
  std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > nqrot;
  // constraint indices
  std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind;
  // constraint local translation 
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ctrans;
  // constraint local rotation as quaternion
  std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > cqrot;
  // constraint covariance
  std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > cvar;
  // tracks
  std::vector<struct tinfo> tracks;


  ReadSPAFile(fin,ntrans,nqrot,cind,ctrans,cqrot,cvar,tracks);

  cout << "# [ReadSPAFile] Found " << (int)ntrans.size() << " nodes and " 
       << (int)cind.size() << " constraints" << endl;

  if (nnodes < 0)
    nnodes = ntrans.size();
  if (nnodes > (int)ntrans.size()) nnodes = ntrans.size();

  // system
  SysSPA spa;
  spa.verbose=false;
  //  spa.useCholmod(true);
  spa.csp.useCholmod = true;

  // add first node
  Node nd;

  // rotation
  Quaternion<double> frq;
  frq.coeffs() = nqrot[0];
  frq.normalize();
  if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
  nd.qrot = frq.coeffs();

  // translation
  Vector4d v;
  v.head(3) = ntrans[0];
  v(3) = 1.0;
  nd.trans = v;
  nd.setTransform();        // set up world2node transform
  nd.setDr(true);

  // add to system
  spa.nodes.push_back(nd);

  double cumtime = 0.0;
  //cout << nd.trans.transpose() << endl << endl;

  // add in nodes
  for (int i=0; i<nnodes-1; i+=doiters)
    {
      for (int j=0; j<doiters; j++)
        if (i+j+1 < nnodes)
          addnode(spa, i+j+1, ntrans, nqrot, cind, ctrans, cqrot, cvar);

      long long t0, t1;

      spa.nFixed = 1;           // one fixed frame

      t0 = utime();
      //      spa.doSPA(1,1.0e-4,SBA_SPARSE_CHOLESKY);
      spa.doSPA(1,1.0e-4,SBA_BLOCK_JACOBIAN_PCG,1.0e-8,15);
      t1 = utime();
      cumtime += t1 - t0;

      cerr 
        << "nodes= " << spa.nodes.size()
        << "\tedges= " << spa.p2cons.size()
        << "\t chi2= " << spa.calcCost()
        << "\t time= " << (t1 - t0) * 1e-6
        << "\t cumTime= " << cumtime*1e-6
        << endl;


    }

  printf("[TestSPA] Compute took %0.2f ms/node, total squared cost %0.2f ms\n", 0.001*(double)cumtime/(double)nnodes, cumtime*0.001);

  //ofstream ofs("sphere-ground.txt");
  //for (int i=0; i<(int)ntrans.size(); i++)
    //ofs << ntrans[i].transpose() << endl;
  //ofs.close();

  //ofstream ofs2("sphere-opt.txt");
  //for (int i=0; i<(int)spa.nodes.size(); i++)
    //ofs2 << spa.nodes[i].trans.transpose().head(3) << endl;
  //ofs2.close();

  //spa.writeSparseA("sphere-sparse.txt",true);

  return 0;
}
