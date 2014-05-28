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
#include "sba/spa2d.h"
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
addnode(SysSPA2d &spa, int n, 
	// node translation
	std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ntrans,
	// node rotation
	std::vector< double > arots,
	// constraint indices
	std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind,
	// constraint local translation 
	std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ctrans,
	// constraint local rotation as quaternion
	std::vector< double > carot,
	// constraint precision
	std::vector< Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3> > > cvar)

{
  Node2d nd1;

  nd1.arot = arots[n];
  nd1.trans.head(2) = ntrans[n];
  nd1.trans(2) = 1.0;

  // add in to system
  nd1.setTransform();		// set up world2node transform
  nd1.setDr();
  spa.nodes.push_back(nd1);

  //  cout << nd0.trans.transpose() << endl << nd1.trans.transpose() << endl << endl;

  // add in constraints
  for (int i=0; i<(int)ctrans.size(); i++)
    {
      Con2dP2 con;
      con.ndr = cind[i].x();
      con.nd1 = cind[i].y();

      if ((con.ndr == n && con.nd1 <= n-1) ||
          (con.nd1 == n && con.ndr <= n-1))
        {
          con.tmean = ctrans[i];
          con.amean = carot[i];
          con.prec = cvar[i];

          spa.p2cons.push_back(con);
        }
    }
}



//
// first argument is the name of input file.
// files are in Freiburg's VERTEX2/EDGE2 format
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
  int nnodes = 0;

  if (argc > 2)
    nnodes = atoi(argv[2]);

  int doiters = 10;
  if (argc > 3)
    doiters = atoi(argv[3]);

  fin = argv[1];

  // node translation
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ntrans;
  // node rotation
  std::vector< double > arots;
  // constraint indices
  std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind;
  // constraint local translation 
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ctrans;
  // constraint local rotation as quaternion
  std::vector< double > carot;
  // constraint precision
  std::vector< Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3> > > cvar;
  // scans
  std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > > scans;

  ReadSPA2dFile(fin,ntrans,arots,cind,ctrans,carot,cvar,scans);

  cout << "# [ReadSPA2dFile] Found " << (int)ntrans.size() << " nodes and " 
       << (int)cind.size() << " constraints" << endl;


  // system
  SysSPA2d spa;
  spa.verbose=false;
  spa.print_iros_stats=false;
  //  spa.useCholmod(false);
  spa.useCholmod(true);


  // use max nodes if we haven't specified it
  if (nnodes == 0) nnodes = ntrans.size();
  if (nnodes > (int)ntrans.size()) nnodes = ntrans.size();

  // add first node
  Node2d nd;

  // rotation
  nd.arot = arots[0];
  // translation
  Vector3d v;
  v.head(2) = ntrans[0];
  v(2) = 1.0;
  nd.trans = v;

  double cumtime = 0.0;
  //cout << nd.trans.transpose() << endl << endl;

  // add to system
  nd.setTransform();            // set up world2node transform
  nd.setDr();
  spa.nodes.push_back(nd);
  // add in nodes
  for (int i=0; i<nnodes-1; i+=doiters)
    {
      for (int j=0; j<doiters; j++)
        addnode(spa, i+j+1, ntrans, arots, cind, ctrans, carot, cvar);

      // cout << "[SysSPA2d] Using " << (int)spa.nodes.size() << " nodes and " 
      //      << (int)spa.p2cons.size() << " constraints" << endl;

      long long t0, t1;

      spa.nFixed = 1;           // one fixed frame

      t0 = utime();
      //      spa.doSPA(1,1.0e-4,SBA_SPARSE_CHOLESKY);
      spa.doSPA(1,1.0e-4,SBA_BLOCK_JACOBIAN_PCG,1.0e-8,15);
      t1 = utime();
      cumtime += t1 - t0;
      if (i%100 == 0) 
        {
          cout << "[SPA2D] iteration: " << i << " squared cost " << spa.errcost << endl;
          cerr << i << " " << cumtime*.001 << " " << spa.errcost << endl;
        }
    }



  printf("[TestSPA2D] Compute took %0.2f ms/node, total time %0.2f ms; error %0.2f\n", 0.001*(double)cumtime/(double)nnodes, cumtime*0.001, spa.errcost);
  // printf("[TestSPA] Accepted iterations: %d\n", niters);
  // printf("[TestSPA] Distance cost: %0.3f m rms\n", sqrt(spa.calcCost(true)/(double)spa.p2cons.size()));

    // if (verbose()){
    //   cerr << "iteration= " << niters 
    // 	   << "\t chi2= " << spa.calcCost();
    //     << "\t time= " << 0.0
    //     << "\t cumTime= " << 0.0
    //     << "\t kurtChi2= " << this->kurtChi2()
    //     << endl;
    // }



#if 0
  ofstream ofs("opt2d-ground.txt");
  for (int i=0; i<(int)ntrans.size(); i++)
    ofs << ntrans[i].transpose() << endl;
  ofs.close();

  ofstream ofs2("opt2d-opt.txt");
  for (int i=0; i<(int)spa.nodes.size(); i++)
    ofs2 << spa.nodes[i].trans.transpose().head(2) << endl;
  ofs2.close();
#endif

  //  spa.writeSparseA("sphere-sparse.txt",true);

  return 0;
}
