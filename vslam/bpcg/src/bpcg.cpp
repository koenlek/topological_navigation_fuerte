
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
// block preconditioned conjugate gradient
// 6x6 blocks at present, should be templatized
//

#include "bpcg/bpcg.h"


namespace sba
{

  //
  // matrix multiply of compressed column storage + diagonal blocks by a vector
  //

  // need to specify alignment of vin/vout segments, can only do it in Eigen3


void
mMV(vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > &diag,
    vector< map<int,Matrix<double,6,6>, less<int>, aligned_allocator<Matrix<double,6,6> > > > &cols,
    const VectorXd &vin,
    VectorXd &vout)
  {
    // loop over diag entries
    //    for (int i=0; i<(int)diag.size(); i++)

    // loop over off-diag entries
    if (cols.size() > 0)
    for (int i=0; i<(int)cols.size(); i++)
      {
	vout.segment<6>(i*6) = diag[i]*vin.segment<6>(i*6); // only works with cols ordering

	map<int,Matrix<double,6,6>, less<int>, 
	  aligned_allocator<Matrix<double,6,6> > > &col = cols[i];
	if (col.size() > 0)
	  {
	    map<int,Matrix<double,6,6>, less<int>, 
	      aligned_allocator<Matrix<double,6,6> > >::iterator it;
	    for (it = col.begin(); it != col.end(); it++)
	      {
		int ri = (*it).first; // get row index
		const Matrix<double,6,6> &M = (*it).second; // matrix
		vout.segment<6>(i*6)  += M.transpose()*vin.segment<6>(ri*6);
		vout.segment<6>(ri*6) += M*vin.segment<6>(i*6);
	      }
	  }
      }
   }

void
mD(vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > &diag,
    VectorXd &vin,
   VectorXd &vout)
{
    // loop over diag entries
    for (int i=0; i<(int)diag.size(); i++)
      vout.segment<6>(i*6) = diag[i]*vin.segment<6>(i*6);
}


//
// jacobi-preconditioned block conjugate gradient
// returns number of iterations
// stopping criteria <tol> is relative reduction in residual norm squared
//

static double residual = 0.0;   // shouldn't do this, it leaves state...

int
bpcg_jacobi(int iters, double tol,
	    vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > &diag,
	    vector< map<int,Matrix<double,6,6>, less<int>, aligned_allocator<Matrix<double,6,6> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol,
	    bool verbose)
{
  // set up local vars
  VectorXd r,d,q,s;
  int n = diag.size();
  int n6 = n*6;
  r.setZero(n6);
  d.setZero(n6);
  q.setZero(n6);
  s.setZero(n6);

  // set up Jacobi preconditioner
  vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > J;
  J.resize(n);
  for (int i=0; i<n; i++)
    {
      J[i] = diag[i].inverse();
      //      J[i].setIdentity();
    }

  int i;
  r = b;
  mD(J,r,d);
  double dn = r.dot(d);
  double d0 = tol*dn;
  if (abstol)			// change tolerances
    {
      if (residual > d0) d0 = residual;
    }

  for (i=0; i<iters; i++)
    {
      if (verbose && 0)
	cout << "[BPCG] residual[" << i << "]: " << dn << " < " << d0 << endl;
      if (dn < d0) break;	// done
      mMV(diag,cols,d,q);
      double a = dn / d.dot(q);
      x += a*d;
      // TODO: reset residual here every 50 iterations
      r -= a*q;
      mD(J,r,s);
      double dold = dn;
      dn = r.dot(s);
      double ba = dn / dold;
      d = s + ba*d;
    }

  
  if (verbose)
    cout << "[BPCG] residual[" << i << "]: " << dn << endl;
  residual = dn/2.0;
  return i;
}


//
// dense algorithm
//

int
bpcg_jacobi_dense(int iters, double tol,
		  MatrixXd &M,
		  VectorXd &x,
		  VectorXd &b)
{
  // set up local vars
  VectorXd r,ad,d,q,s;
  int n6 = M.cols();
  int n = n6/6;
  r.setZero(n6);
  ad.setZero(n6);
  d.setZero(n6);
  q.setZero(n6);
  s.setZero(n6);

  // set up Jacobi preconditioner
  vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > J;
  J.resize(n);
  for (int i=0; i<n; i++)
    {
      J[i] = M.block(i*6,i*6,6,6).inverse();
      //      J[i].setIdentity();
    }

  int i;
  r = b;
  mD(J,r,d);
  double dn = r.dot(d);
  double d0 = dn;

  for (i=0; i<iters; i++)
    {
      cout << "residual[" << i << "]: " << dn << endl;
      if (dn < tol*d0) break; // done
      
      q = M*d;
      double a = dn / d.dot(q);
      x += a*d;
      // TODO: reset residual here every 50 iterations
      r -= a*q;
      mD(J,r,s);
      double dold = dn;
      dn = r.dot(s);
      double ba = dn / dold;
      d = s + ba*d;
    }

  return i;
}


///
/// 3x3 case; templatize already!
///

//
// matrix multiply of compressed column storage + diagonal blocks by a vector
//

// need to specify alignment of vin/vout segments, can only do it in Eigen3

static vector<int> vcind, vrind;
static vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > vcols;
static int ahead;

double
mMV3(vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > &diag,
    vector< map<int,Matrix<double,3,3>, less<int>, aligned_allocator<Matrix<double,3,3> > > > &cols,
    const VectorXd &vin,
    VectorXd &vout)
  {
    double sum = 0;

#if 0
    // loop over off-diag entries
    if (cols.size() > 0)
    for (int i=0; i<(int)cols.size(); i++)
      {
	vout.segment<3>(i*3) = diag[i]*vin.segment<3>(i*3); // only works with cols ordering

	map<int,Matrix<double,3,3>, less<int>, 
	  aligned_allocator<Matrix<double,3,3> > > &col = cols[i];
	if (col.size() > 0)
	  {
	    map<int,Matrix<double,3,3>, less<int>, 
	      aligned_allocator<Matrix<double,3,3> > >::iterator it;
	    for (it = col.begin(); it != col.end(); it++)
	      {
		int ri = (*it).first; // get row index
		const Matrix<double,3,3> &M = (*it).second; // matrix
		vout.segment<3>(i*3)  += M.transpose()*vin.segment<3>(ri*3);
		vout.segment<3>(ri*3) += M*vin.segment<3>(i*3);
	      }
	  }
      }
#else

    // linear storage for matrices
    // lookahead doesn't help

    // loop over off-diag entries
    if (cols.size() > 0)
    for (int i=0; i<(int)cols.size(); i++)
      vout.segment<3>(i*3) = diag[i]*vin.segment<3>(i*3); // only works with cols ordering

    for (int i=0; i<(int)vcind.size(); i++)
      {
        int ri = vrind[i];
        int ii = vcind[i];
        const Matrix<double,3,3> &M = vcols[i];
        int ari = vrind[i+ahead];
        vout.segment<3>(ii*3)  += M.transpose()*vin.segment<3>(ri*3);
        vout.segment<3>(ri*3) += M*vin.segment<3>(ii*3);
        sum += vout[3*ari] + vin[3*ari];
      }
#endif

    return sum;
  }

void
mD3(vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > &diag,
    VectorXd &vin,
   VectorXd &vout)
{
    // loop over diag entries
    for (int i=0; i<(int)diag.size(); i++)
      vout.segment<3>(i*3) = diag[i]*vin.segment<3>(i*3);
}


//
// jacobi-preconditioned block conjugate gradient
// returns number of iterations
// stopping criteria <tol> is relative reduction in residual norm squared
//

int
bpcg_jacobi3(int iters, double tol,
	    vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > &diag,
	    vector< map<int,Matrix<double,3,3>, less<int>, aligned_allocator<Matrix<double,3,3> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol,
	    bool verbose)
{
  // lookahead
  ahead = 8;

  // set up local vars
  VectorXd r,d,q,s;
  int n = diag.size();
  int n3 = n*3;
  r.setZero(n3);
  d.setZero(n3);
  q.setZero(n3);
  s.setZero(n3);

  vcind.clear();
  vrind.clear();
  vcols.clear();

  // set up alternate rep for sparse matrix
  for (int i=0; i<(int)cols.size(); i++)
    {
      map<int,Matrix<double,3,3>, less<int>, 
	aligned_allocator<Matrix<double,3,3> > > &col = cols[i];
      if (col.size() > 0)
	{
	  map<int,Matrix<double,3,3>, less<int>, 
	    aligned_allocator<Matrix<double,3,3> > >::iterator it;
          for (it = col.begin(); it != col.end(); it++)
	    {
	      int ri = (*it).first; // get row index
	      vrind.push_back(ri);
	      vcind.push_back(i);
	      vcols.push_back((*it).second);
	    }
        }
    }

  // lookahead padding
  for (int i=0; i<ahead; i++)
    vrind.push_back(0);

  

  // set up Jacobi preconditioner
  vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > J3;
  J3.resize(n);
  for (int i=0; i<n; i++)
    {
      J3[i] = diag[i].inverse();
      //      J3[i].setIdentity();
    }

  int i;
  r = b;
  mD3(J3,r,d);
  double dn = r.dot(d);
  double d0 = tol*dn;
  if (abstol)			// change tolerances
    {
      if (residual > d0) d0 = residual;
    }

  for (i=0; i<iters; i++)
    {
      if (verbose)
	cout << "[BPCG] residual[" << i << "]: " << dn << " < " << d0 << " " << tol << endl;
      if (dn < d0) break;	// done
      mMV3(diag,cols,d,q);
      double a = dn / d.dot(q);
      x += a*d;
      // TODO: reset residual here every 50 iterations
      r -= a*q;
      mD3(J3,r,s);
      double dold = dn;
      dn = r.dot(s);
      double ba = dn / dold;
      d = s + ba*d;
    }

  
  residual = dn/2.0;

  if (verbose)
    cout << "[BPCG] residual[" << i << "]: " << dn << " " << residual << endl;
  return i;
}


//
// dense algorithm
//

int
bpcg_jacobi_dense3(int iters, double tol,
		  MatrixXd &M,
		  VectorXd &x,
		  VectorXd &b)
{
  // set up local vars
  VectorXd r,ad,d,q,s;
  int n3 = M.cols();
  int n = n3/3;
  r.setZero(n3);
  ad.setZero(n3);
  d.setZero(n3);
  q.setZero(n3);
  s.setZero(n3);

  // set up Jacobi preconditioner
  vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > J3;
  J3.resize(n);
  for (int i=0; i<n; i++)
    {
      J3[i] = M.block(i*3,i*3,3,3).inverse();
      //      J3[i].setIdentity();
    }

  int i;
  r = b;
  mD3(J3,r,d);
  double dn = r.dot(d);
  double d0 = dn;

  for (i=0; i<iters; i++)
    {
      cout << "residual[" << i << "]: " << dn << endl;
      if (dn < tol*d0) break; // done
      
      q = M*d;
      double a = dn / d.dot(q);
      x += a*d;
      // TODO: reset residual here every 50 iterations
      r -= a*q;
      mD3(J3,r,s);
      double dold = dn;
      dn = r.dot(s);
      double ba = dn / dold;
      d = s + ba*d;
    }

  return i;
}



} // end namespace sba

