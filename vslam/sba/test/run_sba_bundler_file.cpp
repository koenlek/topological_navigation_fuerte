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

// run an (undistorted) Bundler file through SBA
// files are in ~/devel/sba-data/venice

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
// test cholmod timing
//

#include <time.h>
#define CPUTIME ((double) (clock ( )) / CLOCKS_PER_SEC)

#ifdef SBA_CHOLMOD
void cholmod_timing(char *fA, char *fB)
{
    FILE *ff = NULL ;
    FILE *fb = NULL ;
    ff = fopen(fA,"r");
    fb = fopen(fB,"r");

    cholmod_sparse *A ;
    cholmod_dense *x, *b, *r ;
    cholmod_factor *L ;
    double one [2] = {1,0}, m1 [2] = {-1,0} ; // basic scalars 
    cholmod_common c ;
    cholmod_start (&c) ;			    /* start CHOLMOD */
    printf("Reading %s\n",fA);
    A = cholmod_read_sparse (ff, &c) ;              /* read in a matrix */
    cholmod_print_sparse (A, (char *)"A", &c) ; /* print the matrix */
    if (A == NULL || A->stype == 0)		    /* A must be symmetric */
    {
	cholmod_free_sparse (&A, &c) ;
	cholmod_finish (&c) ;
	return ;
    }
    printf("Reading %s\n",fB);
    if (fb)
      b = cholmod_read_dense(fb, &c);
    else
      b = cholmod_ones (A->nrow, 1, A->xtype, &c) ; /* b = ones(n,1) */
    double t0 = CPUTIME;
    L = cholmod_analyze (A, &c) ;		    /* analyze */
    cholmod_factorize (A, L, &c) ;		    /* factorize */
    x = cholmod_solve (CHOLMOD_A, L, b, &c) ;	    /* solve Ax=b */
    double t1 = CPUTIME;
    printf("Time: %12.4f \n", t1-t0);
    r = cholmod_copy_dense (b, &c) ;		    /* r = b */
    cholmod_sdmult (A, 0, m1, one, x, r, &c) ;	    /* r = r-Ax */
    printf ("norm(b-Ax) %8.1e\n",
	    cholmod_norm_dense (r, 0, &c)) ;	    /* print norm(r) */
    cholmod_free_factor (&L, &c) ;		    /* free matrices */
    cholmod_free_sparse (&A, &c) ;
    cholmod_free_dense (&r, &c) ;
    cholmod_free_dense (&x, &c) ;
    cholmod_free_dense (&b, &c) ;
    cholmod_finish (&c) ;			    /* finish CHOLMOD */
}
#endif

//
// first argument is the name of input file, Bundler format
//    expects good focal length and distortion correction
// runs sba
//

int main(int argc, char **argv)
{
  char *fin;

  if (argc < 2)
    {
      cout << "Arguments are:  <input filename> [<min conn pts>]" << endl;
      return -1;
    }

  int minpts = 0;
  if (argc > 2)
    minpts = atoi(argv[2]);

  fin = argv[1];


  // construct an SBA system
  SysSBA sys;

  // set up system
  sba::readBundlerFile(fin,sys);
  

  if (minpts > 0)
    {
      int nrem = sys.reduceLongTracks(minpts); // tracks greater than minpts size are removed
    //      sys.remExcessTracks(minpts);
      cout << "Split " << nrem << " / " << sys.tracks.size() << " tracks" << endl; 
    }

  int nprjs = sys.countProjs();

  cout << "Calculating cost" << endl;
  double cost = sys.calcCost();
  cout << "Initial squared cost: " << cost << ",  which is " << sqrt(cost/nprjs) << " rms pixels per projection"  << endl;

  sys.nFixed = 1;
  sys.printStats();
  sys.csp.useCholmod = true;
  sys.huber = 4.0;


#if 0
  //  sba::writeLourakisFile((char *)"bra-340", sys);
  //  cout << endl << "Wrote SBA system in Lourakis format" << endl << endl;
  sba::writeGraphFile((char *)"output.g2o", sys);
  cout << endl << "Wrote SBA system in g2o format" << endl << endl;
#endif

#if 0
  cout << endl;

  cout << "Bad projs (> 100 pix): " << sys.countBad(100.0) 
       << "  Cost without: " << sqrt(sys.calcCost(100.0)/nprjs) << endl;
  cout << "Bad projs (> 20 pix): " << sys.countBad(20.0)
       << "  Cost without: " << sqrt(sys.calcCost(20.0)/nprjs) << endl;
  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0)
       << "  Cost without: " << sqrt(sys.calcCost(10.0)/nprjs) << endl;
  int n = sys.removeBad(20.0);
  cout << "Removed " << n << " projs with >10px error" << endl;
  sys.printStats();
#endif

#if 0
  sys.doSBA(2);
  sys.setupSys(0.0);
  sys.writeSparseA((char *)"A819.venice");
#endif

  //  sys.setConnMat(minpts);
  //  sys.setConnMatReduced(minpts);             // finds spanning tree


#if 0
  // save sparsity pattern 
  cout << "[SBAsys] Saving sparsity pattern in <sparsity.txt>" << endl;
  sys.doSBA(1,1e-3,0);
  FILE *fd = fopen("sparsity.txt","w");
  int m = sys.B.size();
  for (int i=0; i<m; i+=6)
    {
      for (int j=0; j<m; j+=6)
        if (sys.A(i,j) != 0.0)
          fprintf(fd,"1 ");
        else
          fprintf(fd,"0 ");  
      fprintf(fd,"\n");
    }
  fclose(fd);
#endif


  //  sys.doSBA(10,1e-4,SBA_SPARSE_CHOLESKY);
  sys.doSBA(10,1e-4,SBA_BLOCK_JACOBIAN_PCG,1e-8,200);

  cout << endl;

  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) 
       << "  Cost without: " << sqrt(sys.calcCost(10.0)/nprjs) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0)
       << "  Cost without: " << sqrt(sys.calcCost( 5.0)/nprjs) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0)
       << "  Cost without: " << sqrt(sys.calcCost( 2.0)/nprjs) << endl << endl;

  sys.doSBA(10,1e-3,SBA_SPARSE_CHOLESKY);
  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0) << endl << endl;

  //  sys.removeBad(4.0);
  //  cout << "Removed projs with >4px error" << endl;

  sys.doSBA(10);
  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0) << endl << endl;

  sys.doSBA(10);
  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0) << endl << endl;

  return 0;
}
