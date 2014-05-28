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

// convert a file in Freiburg's VERTEX / EDGE format into a set of constraints

#include "sba/read_spa.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <sys/time.h>

using namespace std;
using namespace Eigen;

//
// read in an SPA file
//

//
// Format:
//  # XXXX  - comment line
//  VERTEX3 n xyz rpy  - node line
//  EDGE3 n1 n2 xyz rpy - edge line
//

// NOTE: assumes vertices are in order starting with index 0

// makes a quaternion from fixed Euler RPY angles
// see the Wikipedia article on Euler anlges
void make_qrot(double rr, double rp, double ry, Vector4d &v)
{
  double sr = sin(rr/2.0);
  double cr = cos(rr/2.0);
  double sp = sin(rp/2.0);
  double cp = cos(rp/2.0);
  double sy = sin(ry/2.0);
  double cy = cos(ry/2.0);
  v[0] = sr*cp*cy - cr*sp*sy;   // qx
  v[1] = cr*sp*cy + sr*cp*sy;   // qy
  v[2] = cr*cp*sy - sr*sp*cy;   // qz
  v[3] = cr*cp*cy + sr*sp*sy;   // qw
}

// cv is upper triangular
void make_covar(double *cv, Matrix<double,6,6> &m)
{
  m.setZero();

  int i = 0;
  m(0,0) = cv[i++];
  m(0,1) = cv[i++];
  m(0,2) = cv[i++];
  m(0,3) = cv[i++];
  m(0,4) = cv[i++];
  m(0,5) = cv[i++];

  m(1,1) = cv[i++];
  m(1,2) = cv[i++];
  m(1,3) = cv[i++];
  m(1,4) = cv[i++];
  m(1,5) = cv[i++];

  m(2,2) = cv[i++];
  m(2,3) = cv[i++];
  m(2,4) = cv[i++];
  m(2,5) = cv[i++];

  m(3,3) = cv[i++];
  m(3,4) = cv[i++];
  m(3,5) = cv[i++];

  m(4,4) = cv[i++];
  m(4,5) = cv[i++];

  m(5,5) = cv[i++];

  // make symmetric
  Matrix<double,6,6> mt = m.transpose();
  mt.diagonal().setZero();
  m = m+mt;
}


int
ReadSPAFile(char *fin,          // input file
            // node translation
            std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ntrans,
            // node rotation
            std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &nqrot,
            // constraint indices
            std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,
            // constraint local translation 
            std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ctrans,
            // constraint local rotation as quaternion
            std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &cqrot,
            // constraint covariance
            std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > &cvar,
            // track info: point projections, see format description in IntelSeattle files
            std::vector<struct tinfo> &tracks
            )
{
  ifstream ifs(fin);
  if (ifs == NULL)
    {
      cout << "Can't open file " << fin << endl;
      return -1;
    }
  ifs.precision(10);

  // loop over lines
  string line;
  int nline = 0;
  bool first = true;
  while (getline(ifs,line))
    {
      nline++;
      stringstream ss(line);    // make a string stream
      string type;
      ss >> type;
      size_t pos = type.find("#");
      if (pos != string::npos)
        continue;               // comment line

      if (type == "VERTEX3")    // have a vertex
        {
          int n;
          double tx,ty,tz,rr,rp,ry;
          if (!(ss >> n >> tx >> ty >> tz >> rr >> rp >> ry))
            {
              cout << "[ReadSPA] Bad VERTEX3 at line " << nline << endl;
              return -1;
            }
          ntrans.push_back(Vector3d(tx,ty,tz));
          Vector4d v;
          make_qrot(rr,rp,ry,v);
          nqrot.push_back(v);
        }

      if (type == "EDGE3")      // have an edge
        {
          int n1,n2;
          double tx,ty,tz,rr,rp,ry;
          double cv[21];

          // indices and measurement
          if (!(ss >> n1 >> n2 >> tx >> ty >> tz >> rr >> rp >> ry))
            {
              cout << "[ReadSPA] Bad EDGE3 at line " << nline << endl;
              return -1;
            }
          cind.push_back(Vector2i(n1,n2));
          ctrans.push_back(Vector3d(tx,ty,tz));
          Vector4d v;
          make_qrot(rr,rp,ry,v);
          cqrot.push_back(v);

          // covar
          if (!(ss >> cv[0] >> cv[1] >> cv[2] >> cv[3] >> cv[4] 
                >> cv[5] >> cv[6] >> cv[7] >> cv[8] >> cv[9] 
                >> cv[10] >> cv[11] >> cv[12] >> cv[13] >> cv[14] 
                >> cv[15] >> cv[16] >> cv[17] >> cv[18] >> cv[19] >> cv[20]))
            {
              cout << "[ReadSPA] Bad EDGE3 at line " << nline << endl;
              return -1;
            }
          Matrix<double,6,6> m;
          make_covar(cv,m);
          if (first)
            {
              //cout << endl;
              //for (int j=0; j<21; j++);
                //cout << cv[j] << " ";
              //cout << endl << endl << << m << endl;
              first = false;
            }
          cvar.push_back(m);
        }


      if (type == "TRACK32")      // have a point projection pair
        {
          struct tinfo tt;
          int pi,fi,fpi;
          double tx,ty,tz,u,v;

          // indices and measurement
          if (!(ss >> pi >> fi >> fpi >> tx >> ty >> tz >> u >> v))
            {
              cout << "[ReadSPA] Bad TRACK32 at line " << nline << endl;
              return -1;
            }
          tt.pn = pi;
          tt.fn0 = fi;
          tt.fpn0 = fpi;
          tt.x0 = tx;
          tt.y0 = ty;
          tt.z0 = tz;
          tt.u0 = u;
          tt.v0 = v;

          // second projection of pair
          if (!(ss >> fi >> fpi >> tx >> ty >> tz >> u >> v))
            {
              cout << "[ReadSPA] Bad TRACK32 at line " << nline << endl;
              return -1;
            }
          tt.fn1 = fi;
          tt.fpn1 = fpi;
          tt.x1 = tx;
          tt.y1 = ty;
          tt.z1 = tz;
          tt.u1 = u;
          tt.v1 = v;

          tracks.push_back(tt);

        }

    }

  return 0;
}


// 2D code

// cv is upper triangular
void make_covar_2d(double *cv, Matrix<double,3,3> &m, bool useFreiburg)
{
  m.setZero();

  int i = 0;
  if (useFreiburg)
    {
      m(0,0) = cv[i++];
      m(0,1) = cv[i++];
      m(1,1) = cv[i++];

      m(2,2) = cv[i++];
      m(0,2) = cv[i++];
      m(1,2) = cv[i++];
    }
  else
    {
      m(0,0) = cv[i++];
      m(0,1) = cv[i++];
      m(0,2) = cv[i++];

      m(1,1) = cv[i++];
      m(1,2) = cv[i++];

      m(2,2) = cv[i++];
    }

  // make symmetric
  Matrix<double,3,3> mt = m.transpose();
  mt.diagonal().setZero();
  m = m+mt;
}


int
ReadSPA2dFile(char *fin,          // input file
            // node translation
            std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ntrans,
            // node rotation
            std::vector< double > &narot,
            // constraint indices
            std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,
            // constraint local translation 
            std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ctrans,
            // constraint local rotation as angle
            std::vector< double > &carot,
            // constraint covariance
            std::vector< Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3> > > &cvar,
            // scan points
            std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > > &scans
            )
{
  ifstream ifs(fin);
  if (ifs == NULL)
    {
      cout << "Can't open file " << fin << endl;
      return -1;
    }
  ifs.precision(10);

  VectorXi ind;                 // indices
  ind.setConstant(1000000,-1);  // invalid 
  int cur = 0;                  // current index
  bool useFreiburg = false;     // for Freiburg odd covariances

  // loop over lines
  string line;
  int nline = 0;
  bool first = true;
  while (getline(ifs,line))
    {
      nline++;
      stringstream ss(line);    // make a string stream
      string type;
      ss >> type;
      size_t pos = type.find("#");
      if (pos != string::npos)
        {
          pos = type.find("#FCOVAR");
          if (pos != string::npos)
            {
              useFreiburg = true;
              cout << "Using Freiburg covariances" << endl;
            }
          continue;             // comment line
        }

      if (type == "VERTEX2")    // have a vertex
        {
          int n;
          double tx,ty,rr;
          if (!(ss >> n >> tx >> ty >> rr))
            {
              cout << "[ReadSPA2d] Bad VERTEX2 " << n << " at line " << nline << endl;
              return -1;
            }
          ntrans.push_back(Vector2d(tx,ty));
          narot.push_back(rr);
          ind[n] = cur++;
        }

      if (type == "EDGE2")      // have an edge
        {
          int n1,n2;
          double tx,ty,rr;
          double cv[6];

          // indices and measurement
          if (!(ss >> n1 >> n2 >> tx >> ty >> rr))
            {
              cout << "[ReadSPA2d] Bad EDGE2 at line " << nline << endl;
              return -1;
            }

          n1 = ind[n1];
          n2 = ind[n2];

          if (n1 < 0 || n2 < 0)
            {
              cout << "[ReadSPA2d] Bad EDGE2 indices at line " << nline << endl << line << endl;
              return -1;
            }

          cind.push_back(Vector2i(n1,n2));
          ctrans.push_back(Vector2d(tx,ty));
          carot.push_back(rr);

          // covar
          if (!(ss >> cv[0] >> cv[1] >> cv[2] >> cv[3] >> cv[4] 
                >> cv[5]))
            {
              cout << "[ReadSPA2d] Bad EDGE2 at line " << nline << endl;
              return -1;
            }
          Matrix<double,3,3> m;
          make_covar_2d(cv,m,useFreiburg);
          if (first)
            {
              // cout << endl;
              // for (int j=0; j<6; j++)
              //   cout << cv[j] << " ";
              // cout << endl << endl << m << endl;
              first = false;
            }
          cvar.push_back(m);
        }

      if (type == "POINT2")     // have a scan
        {
          int n;
          double px,py;
          if (!(ss >> n ))
            {
              cout << "[ReadSPA2d] Bad POINT2 size " << n << " at line " << nline << endl;
              return -1;
            }
          if (n >= (int)scans.size()) // n is the scan number
            scans.resize(n+1);

          // coords of scan points are in world system!!!!
          double a = narot[n];  // angle of node for this scan
          Vector3d tr;
          tr.head(2) = ntrans[n]; // translation
          tr[2] = 1.0;
          Matrix<double,2,3> w2n;
          w2n(0,0) = w2n(1,1) = cos(a);
          w2n(0,1) = sin(a);
          w2n(1,0) = -w2n(0,1);
          w2n.col(2).setZero();
          w2n.col(2) = -w2n*tr;

          std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > scan;
          string str;
          getline(ss,str);
          istringstream sss(str);
          while ((sss >> px >> py))
            {
              Vector3d pw(px,py,1.0);
              Vector2d pn = w2n*pw;
              scan.push_back(pn);
            }
          scans[n] = scan;
        }
    }

  return 0;
}

