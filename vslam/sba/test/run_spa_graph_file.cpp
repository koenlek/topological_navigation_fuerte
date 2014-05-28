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

// run a graph file through sSBA
// sample files are in data/ directory

#include "sba/sba_file_io.h"
#include "sba/sba.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <sys/time.h>

using namespace std;
using namespace Eigen;
using namespace sba;
using namespace frame_common;


//
// first argument is the name of input file, graph format
// runs sba
//

int main(int argc, char **argv)
{
  char *fin;

  if (argc < 2)
    {
      cout << "Arguments are:  <input filename>" << endl;
      return -1;
    }

  fin = argv[1];

  SysSPA sys;
  readSPAGraphFile(fin, sys);
  //  writeGraphFile("sba-out.graph", sys);

  double cost = sys.calcCost();
  cout << "Initial squared cost: " << cost << endl;
  sys.spanningTree();
  cost = sys.calcCost();
  cout << "Spanning tree squared cost: " << cost << endl;

  sys.nFixed = 1;
  sys.csp.useCholmod = true;
  sys.verbose = true;

  //  sys.doSPA(10,1e-4,SBA_SPARSE_CHOLESKY);
  sys.doSPA(10,1e-4,SBA_BLOCK_JACOBIAN_PCG,1e-8,200);

  return 0;
}
