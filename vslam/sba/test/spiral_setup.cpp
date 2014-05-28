
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

// setup fns for spiral trajectory

#include "sba/sba_setup.h"
using namespace Eigen;
using namespace sba;
using namespace frame_common;

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <sys/time.h>

using namespace std;

static double near = 0.5;
static double far = 10.0;

// ref count points, based on their visible projections
// plus, add them to the SBA system, and set up their indices
static int camn = 0;
static int camp = -1;

int mark_points(SysSBA &sba, Node& ci, vector< Point,Eigen::aligned_allocator<Point>  >& Wpts, vector<int> &Wptref, 
                vector<int> &Wptind)
{
  int ntot = 0;
  int npts = Wpts.size();
  double maxx = 2.0*ci.Kcam(0,2); // cx
  double maxy = 2.0*ci.Kcam(1,2); // cy

  if (camn == camp)
    {
      cout << ci.trans.transpose() << endl;
    }

  for (int i=0; i<npts; i++)
    {
      Vector2d qi;
      Vector3d qw;
      qw = ci.w2n * Wpts[i]; // point in camera coords
      if (qw[2] > near && qw[2] < far)
        {
          ci.project2im(qi,Wpts[i]); // point in image coords
          if (qi[0] > 0.5 && qi[0] < maxx &&
              qi[1] > 0.5 && qi[1] < maxy)
            {
              if (camn == camp)
                {
                  cout << "pw: " << Wpts[i].transpose() << endl;
                  cout << "pc: " << qw.transpose() << endl << endl;
                }
              Wptref[i]++;
              if (Wptref[i] == 2)
                {
                  ntot++;
                  Wptind[i] = sba.tracks.size(); // index into Wpts
                  //              cout << ntot << " " << sba.points.size() << endl;
                  sba.addPoint(Wpts[i]);
                }
            }
        }
    }  
  camn++;
  return ntot;
}



// add measurements, assuming ref counts present
// <cind> is the camera/node index
int add_points(SysSBA &sba, double inoise, int cind, 
               vector< Vector4d,Eigen::aligned_allocator<Vector4d> >& Wpts, vector<int> &Wptref,
               vector<int> &Wptind)
{
  int ntot = 0;
  double inoise2 = inoise*0.5;
  int npts = Wpts.size();
  Node &ci = sba.nodes[cind];
  double maxx = 2.0*ci.Kcam(0,2); // cx
  double maxy = 2.0*ci.Kcam(1,2); // cy
  for (int i=0; i<npts; i++)
    {
      if (Wptref[i] < 2) continue; // have to have at least two points
      Vector2d qi;
      Vector3d qw;
      qw = ci.w2n * Wpts[i]; // point in camera coords
      if (qw[2] > near && qw[2] < far)
        {
          ci.project2im(qi,Wpts[i]); // point in image coords
          if (qi[0] > 0.5 && qi[0] < maxx &&
              qi[1] > 0.5 && qi[1] < maxy)
            {
              qi[0] += drand48() * inoise - inoise2; // add noise now
              qi[1] += drand48() * inoise - inoise2;
              //              cout << "Cam and pt indices: " << cind << " " << Wptind[i] << endl;
              sba.addMonoProj(cind, Wptind[i], qi);
              ntot++;
            }
        }
    }  
  return ntot;
}


// elapsed time in microseconds
long long utime()
{
  timeval tv;
  gettimeofday(&tv,NULL);
  long long ts = tv.tv_sec;
  ts *= 1000000;
  ts += tv.tv_usec;
  return ts;
}


// set up spiral camera/point system
void
spiral_setup(SysSBA &sba, CamParams &cpars, vector<Matrix<double,6,1>, Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
             double s_near, double s_far,
             double ptsize, double kfang, double initang, double cycles, 
             double inoise, double pnoise, double qnoise)
{
  // random seed
  unsigned short seed = (unsigned short)utime();
  seed48(&seed);

  // params
  near = s_near;
  far = s_far;
  const double Wsize = 130.0;    // in meters
  const double Wsize2 = Wsize * 0.5;
  int Wvol = (int)(Wsize*Wsize*Wsize+0.5);
  int Npts = Wvol * ptsize;
  vector< Point, Eigen::aligned_allocator<Point> > Wpts; // list of world points
  vector< int > Wptref;         // ref count for points
  vector< int > Wptind;         // index from SBA pts to Wpts
  Wpts.resize(Npts);            // 10K points
  Wptref.resize(Npts);
  Wptind.resize(Npts);
  for (int i=0; i<Npts; i++)
    Wptref[i] = 0;
  
  // fill with random points
  for (int i=0; i<Npts; i++)
    {
      Vector4d qw;
      qw[0] = drand48() * Wsize - Wsize2;
      qw[1] = drand48() * Wsize - Wsize2;
      qw[2] = drand48() * Wsize - Wsize2;
      qw[3] = 1.0;
      Wpts[i] = qw;
    }
  printf("[Spiral] Added %d points to world\n", Npts);

  //
  // spiral trajectory
  //

  double radius = 8.0;          // spiral radius, m
  //  double kfang  = 5.0;              // how far we travel, deg
  double ydiff  = 1.0;          // y spiral diff per cycle
  //  double cycles = 54*kfang/360.0; // number of cycles
  //  double inoise = 0.0;              // measurement noise, pixels
  //  double pnoise = 0.05;             // camera pose noise, meters

  // set up camera poses, and run first pass at ref counting visible points
  //  vector< Matrix<double,6,1> > cps; // set of camera poses, trans + qrot

  double ang = initang;         // initial angle
  double y = -60.0;
  double cycle_kfs = 360.0/kfang;
  int ntot = 0;
  printf("\n[Spiral] Setting up spiral with %d keyframes, %d per cycle\n", (int)(cycles*cycle_kfs), (int)cycle_kfs);
  //  printf("[Spiral] Camera poses:\n");
  
  for (int i=0; i<cycles*cycle_kfs; i++, ang+=kfang*M_PI/180, y+=ydiff/cycle_kfs)
    {
      if (ang > 2*M_PI)
        ang -= 2*M_PI;

      // frame rotation in the world
      Quaternion<double> frq(AngleAxisd(ang+M_PI/2.0,Vector3d(0,1,0).normalized()));
      if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
      double z = radius*cos(ang);
      double x = radius*sin(ang)+radius;
      Vector4d frt(x,y,z,1.0); // frame position in the world

      // make node
      Node nd;
      nd.qrot = frq.coeffs();   
      nd.normRot();
      //      cout << "Quaternion: " << nd.qrot.transpose() << endl;
      nd.trans = frt;
      //      cout << "Translation: " << nd.trans.transpose() << endl << endl;
      nd.setTransform();        // set up world2node transform
      nd.setKcam(cpars);        // set up node2image projection
      nd.setDr(sba.useLocalAngles);

      // save it
      Matrix<double,6,1> pos;
      pos.head(3) = frt.head(3);
      pos.segment<3>(3) = frq.coeffs().head(3);
      cps.push_back(pos);

      // check for points seen by node
      ntot += mark_points(sba,nd,Wpts,Wptref,Wptind);


      // add to SBA
      sba.nodes.push_back(nd);

      // print out camera poses
      //      printf("   %f %f %f %f %f %f\n", x,y,z,0.0,ang+M_PI/2,0.0);
    }
  printf("[Spiral] Found %d referenced points for measurements\n", ntot);


  // loop over cameras, adding measurements
  double totrefs = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      int ret = add_points(sba, inoise, i, Wpts, Wptref, Wptind);
      totrefs += ret;
      //      printf("[Spiral] Added %d measurements for camera %d\n", ret, i);
    }
  double avpts = totrefs / (double)sba.nodes.size();
  printf("[Spiral] Average %0.1f measurements per camera\n", avpts);

  int ntr = 0;
  for (size_t i=0; i<sba.tracks.size(); i++)
    ntr += sba.tracks[i].projections.size();
  double avtracks = ntr / (double)sba.tracks.size();
  printf("[Spiral] Average %0.1f measurements per track\n", avtracks);


  // add error to initial positions
  for (size_t i=0; i<sba.nodes.size(); i++)
    {
      Node &nd = sba.nodes[i];
      if (i>0)          // add error
        {
          Vector4d frt2(drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2,
                        0.0);
          nd.trans += frt2;

          Vector4d frq2(drand48()*qnoise - qnoise/2,
                        drand48()*qnoise - qnoise/2,
                        drand48()*qnoise - qnoise/2,
                        0.0);
          nd.qrot.coeffs() += frq2;
          double nrm = nd.qrot.vec().squaredNorm();
          if (nrm > 0.9999)
            nd.qrot.vec() = -nd.qrot.vec() / (sqrt(nrm)*1.0001);
          nd.normRot();

          nd.setTransform();    // set up world2node transform
          nd.setProjection();   // set up node2image projection
          nd.setDr(sba.useLocalAngles); // set rotational derivatives
        }
    }
 }


//
// add a pose-pose constraint
// for now just use simple precision matrix
//   NOTE: translation has to be about 1/1000 the precision of rotation...
//

int add_p2con(SysSPA &spa, int i, int j, 
              Eigen::Matrix<double,6,6> prec,
              double mpnoise, double mqnoise)
{
  ConP2 con;
  con.ndr = i;
  con.nd1 = j;
  Node nd0 = spa.nodes[i];
  Node nd1 = spa.nodes[j];

  con.prec = prec;

  con.tmean = nd0.w2n * nd1.trans; // translation offset
  Quaternion<double> q0,q1;
  q0 = nd0.qrot;
  q1 = nd1.qrot;
  con.qpmean = (q0.inverse()*q1).inverse();

  con.tmean *= 1.0 + mpnoise;         // scale change

  spa.p2cons.push_back(con);

  //  cout << "Mean of " << con.ndr << "->" << con.nd1 << " is " 
  //       << con.tmean.transpose() << "   " << con.qpmean.coeffs().transpose() << endl;
  //  cout << "Prec of " << con.ndr << "->" << con.nd1 << endl << " is " << con.prec << endl;

  return 0;
}

// set up spiral frame/frame system
void
spa_spiral_setup(SysSPA &spa, bool use_cross_links,
                 vector<Matrix<double,6,1>, Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
                 Matrix<double,6,6> prec, Matrix<double,6,6> vprec,
                 Matrix<double,6,6> a10prec, Matrix<double,6,6> a15prec,
                 double kfang, double initang, double cycles, 
                 double pnoise, double qnoise, double snoise, 
                 double dpnoise, double dqnoise)
{

  // random seed
  unsigned short seed = (unsigned short)utime();
  seed48(&seed);


  //
  // spiral trajectory
  //

  double radius = 8.0;          // spiral radius, m
  //  double kfang  = 5.0;              // how far we travel, deg
  double ydiff  = 1.0;          // y spiral diff per cycle
  //  double cycles = 54*kfang/360.0; // number of cycles
  //  double inoise = 0.0;              // measurement noise, pixels
  //  double pnoise = 0.05;             // camera pose noise, meters

  // set up poses
  //  vector< Matrix<double,6,1> > cps; // set of camera poses, trans + qrot

  double ang = initang;         // initial angle
  double y = -5.0;
  double cycle_kfs = 360.0/kfang;
  int cyc_kfs = (int)(cycle_kfs+.01);
  printf("\n[SPA Spiral] Setting up spiral with %d keyframes, %d per cycle\n", (int)(cycles*cycle_kfs), (int)cycle_kfs);
  //  printf("[SPA Spiral] Camera poses:\n");
  
  for (int i=0; i<cycles*cycle_kfs; i++, ang+=kfang*M_PI/180, y+=ydiff/cycle_kfs)
    {
      if (ang > 2*M_PI)
        ang -= 2*M_PI;

      // frame rotation in the world
      Quaternion<double> frq(AngleAxisd(ang+M_PI/2.0,Vector3d(0,1,0).normalized()));
      if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
      double z = radius*cos(ang);
      double x = radius*sin(ang)+radius;
      Vector4d frt(x,y,z,1.0); // frame position in the world

      // make node
      Node nd;
      nd.qrot = frq.coeffs();   
//      nd.normRot();
      //      cout << "Quaternion: " << nd.qrot.transpose() << endl;
      nd.trans = frt;
      //      cout << "Translation: " << nd.trans.transpose() << endl << endl;
      nd.setTransform();        // set up world2node transform
      nd.setDr(true);

      // save it
      Matrix<double,6,1> pos;
      pos.head(3) = frt.head(3);
      pos.segment<3>(3) = frq.vec();
      cps.push_back(pos);

      // add to SPA
      spa.nodes.push_back(nd);

      // print out camera poses
      //      printf("   %f %f %f %f %f %f\n", x,y,z,0.0,ang+M_PI/2,0.0);
    }




  // add error to initial positions
  qnoise = M_PI*qnoise/180.0; // convert to radians
  Vector3d tinc;
  tinc = spa.nodes[0].w2n * spa.nodes[1].trans;
  double vinc = 1.0;
  for (int i=0; i<(int)spa.nodes.size(); i++)
    {
      Node &nd = spa.nodes[i];

      // add cross-links between cycles
      if (use_cross_links && i+cyc_kfs < (int)spa.nodes.size()-2)
        {
          add_p2con(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          add_p2con(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          add_p2con(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          printf("%d %d \n", i, i+cyc_kfs);
        }


      if (i>=spa.nFixed)        // add error
        {
          // modify tinc each time...
          double inc = 1.0 + snoise * (drand48() - 0.5);
          tinc *= inc;
          vinc *= inc;

          Node &ndp = spa.nodes[i-1];
          Quaternion<double> ndq;
          ndq = ndp.qrot;

          Vector3d frt2(drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2);
          frt2 += tinc;         // ok, randomly modified increment

          //          cout << endl << nd.trans.head(3).transpose() << endl;

          nd.trans.head(3) = ndq.toRotationMatrix()*frt2 + ndp.trans.head(3);

          //          cout << nd.trans.head(3).transpose() << endl;



          Quaternion<double> nq(AngleAxisd(drand48()*qnoise - qnoise/2,
                                           Vector3d(drand48() - 0.5,
                                                    drand48() - 0.5,
                                                    drand48() - 0.5).normalized()));

          nq.normalize();
          ndq = nd.qrot;
          nd.qrot = (nq*ndq).coeffs();
          
          nd.setTransform();    // set up world2node transform
          nd.setDr(true); // set rotational derivatives
        }
    }

  // loop over cameras, adding local measurements
  for (int i=1; i<(int)cps.size(); i++)
    {
      add_p2con(spa, i-1, i, prec, 0.0, 0.0);
      if (i>2)
        add_p2con(spa, i-2, i, a10prec, 0.0, 0.0);
      if (i>3)
        add_p2con(spa, i-3, i, a15prec, 0.0, 0.0);
      //      printf("[SPA Spiral] Added a measurement %d -> %d\n", i-1, i);
    }

  // finally, some initial random noise on node positions and angles
  for (int i=0; i<(int)spa.nodes.size(); i++)
    {
      Node &nd = spa.nodes[i];

      if (i>=spa.nFixed)        // add error
        {
          Vector3d frt2(drand48()*dpnoise - dpnoise/2,
                        drand48()*dpnoise - dpnoise/2,
                        drand48()*dpnoise - dpnoise/2);
          nd.trans.head(3) += frt2;

          dqnoise = M_PI*dqnoise/180.0; // convert to radians
          Quaternion<double> ndq;
          Quaternion<double> nq(AngleAxisd(drand48()*dqnoise - dqnoise/2,
                                           Vector3d(drand48() - 0.5,
                                                    drand48() - 0.5,
                                                    drand48() - 0.5).normalized()));

          nq.normalize();
          ndq = nd.qrot;
          nd.qrot = nq*ndq;
          if (nd.qrot.w() < 0.0)
            nd.qrot.coeffs() = -nd.qrot.coeffs();

          nd.setTransform();    // set up world2node transform
          nd.setDr(true); // set rotational derivatives
        }
    }


 }

//
// add a pose-pose constraint
// for now just use simple precision matrix
//   NOTE: translation has to be about 1/1000 the precision of rotation...
//

int add_p2con2d(SysSPA2d &spa, int i, int j, 
                Eigen::Matrix<double,3,3> prec,
                double mpnoise, double mqnoise)
{
  Con2dP2 con;
  con.ndr = i;
  con.nd1 = j;
  Node2d nd0 = spa.nodes[i];
  Node2d nd1 = spa.nodes[j];

  con.prec = prec;

  con.tmean = nd0.w2n * nd1.trans; // translation offset

  double am = nd1.arot - nd0.arot;
  //  if (am > M_PI) am -= 2.0*M_PI;
  //  if (am < -M_PI) am += 2.0*M_PI;
  con.amean = am;

  con.tmean *= 1.0 + mpnoise;         // scale change

  spa.p2cons.push_back(con);

  //  cout << "Mean of " << con.ndr << "->" << con.nd1 << " is " 
  //       << con.tmean.transpose() << "   " << con.qpmean.coeffs().transpose() << endl;
  //  cout << "Prec of " << con.ndr << "->" << con.nd1 << endl << " is " << con.prec << endl;

  return 0;
}

// set up spiral frame/frame system
void
spa2d_spiral_setup(SysSPA2d &spa, 
                 vector<Matrix<double,3,1>, Eigen::aligned_allocator<Matrix<double,3,1> > > &cps,
                 Matrix<double,3,3> prec, Matrix<double,3,3> vprec,
                 Matrix<double,3,3> a10prec, Matrix<double,3,3> a15prec,
                 double kfang, double initang, double cycles, 
                 double pnoise, double qnoise, double snoise, double dpnoise, double dqnoise)
{

  // random seed
  unsigned short seed = (unsigned short)utime();
  seed48(&seed);


  //
  // spiral trajectory
  //

  double radius = 8.0;          // initial spiral radius, m
  double rdiff  = 1.0;          // spiral radius expansion per cycle

  double ang = initang;         // initial angle
  double cycle_kfs = 360.0/kfang;
  int cyc_kfs = (int)(cycle_kfs+.01);
  printf("\n[SPA Spiral] Setting up spiral with %d keyframes, %d per cycle\n", (int)(cycles*cycle_kfs), (int)cycle_kfs);
  //  printf("[SPA Spiral] Camera poses:\n");
  
  for (int i=0; i<cycles*cycle_kfs; i++, ang+=kfang*M_PI/180, radius+=rdiff/cycle_kfs)
    {
      if (ang > 2*M_PI)
        ang -= 2*M_PI;

      // frame rotation in the world
      double a = ang+M_PI/2.0;
      double z = radius*cos(ang);
      double x = radius*sin(ang)+radius;
      Vector3d frt(x,z,1.0);    // frame position in the world

      // make node
      Node2d nd;
      nd.arot = a;
      nd.trans = frt;
      //      cout << "Translation: " << nd.trans.transpose() << endl << endl;
      nd.normArot();
      nd.setTransform();        // set up world2node transform
      nd.setDr();

      // save it
      Matrix<double,3,1> pos;
      pos.head(2) = frt.head(2);
      pos(2) = nd.arot;
      cps.push_back(pos);

      // add to SPA
      spa.nodes.push_back(nd);

      // print out camera poses
      //      printf("   %f %f %f %f %f %f\n", x,y,z,0.0,ang+M_PI/2,0.0);
    }




  // add error to initial positions
  qnoise = M_PI*qnoise/180.0; // convert to radians
  Vector2d tinc;
  double vinc = 1.0;
  for (int i=0; i<(int)spa.nodes.size(); i++)
    {
      Node2d &nd = spa.nodes[i];

      // add cross-links between cycles
      if (i+cyc_kfs < (int)spa.nodes.size()-2 &&
          (1 || (i%cyc_kfs < 3 || (i+cyc_kfs/2)%cyc_kfs < 3))) // add in cross-links
        {
          add_p2con2d(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          add_p2con(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          add_p2con(spa, i+cyc_kfs, i, vprec, vinc-1.0, 0.0);
          //          printf("%d %d \n", i, i+cyc_kfs);
        }


      if (i>=spa.nFixed)        // add error
        {
          Node2d &ndp = spa.nodes[i-1];
          tinc = ndp.w2n * nd.trans;

          // modify tinc each time...
          double inc = 1.0 + snoise * (drand48() - 0.5);
          tinc *= inc;
          vinc *= inc;


          Vector2d frt2(drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2);
          frt2 += tinc;         // ok, randomly modified increment

          //          cout << endl << nd.trans.head(3).transpose() << endl;

          nd.trans.head(2) = ndp.w2n.block<2,2>(0,0).transpose()*frt2 + ndp.trans.head(2);

          //          cout << nd.trans.head(3).transpose() << endl;


          double a = drand48()*qnoise - qnoise/2;
          nd.arot += a;
          nd.normArot();
          nd.setTransform();    // set up world2node transform
          nd.setDr();           // set rotational derivatives
        }
    }

  // loop over cameras, adding measurements
  for (int i=1; i<(int)cps.size(); i++)
    {
      add_p2con2d(spa, i-1, i, prec, 0.0, 0.0);
      if (i>2)
        add_p2con2d(spa, i-2, i, a10prec, 0.0, 0.0);
      if (i>3)
        add_p2con2d(spa, i-3, i, a15prec, 0.0, 0.0);
      //      if (i > cycle_kfs)        // add in cross-links
      //        add_p2con(spa, i, i-cycle_kfs, vprec, 0.0, 0.0);
      //      printf("[SPA Spiral] Added a measurement %d -> %d\n", i-1, i);
    }

  // finally, some initial random noise on node positions and angles
  for (int i=0; i<(int)spa.nodes.size(); i++)
    {
      Node2d &nd = spa.nodes[i];

      if (i>=spa.nFixed)        // add error
        {
          Vector2d frt2(drand48()*dpnoise - dpnoise/2,
                        drand48()*dpnoise - dpnoise/2);
          nd.trans.head(2) += frt2;

          dqnoise = M_PI*dqnoise/180.0; // convert to radians
          double a = drand48()*dqnoise - dqnoise/2;
          nd.arot += a;
          nd.normArot();
          nd.setTransform();    // set up world2node transform
          nd.setDr();           // set rotational derivatives
        }
    }


 }


// set up spherical camera/point system
// same as Jeong CVPR10 paper

// add point to camera set
// <cinds> is a vector of camera node indices
void
add_sphere_points(SysSBA &sba, double inoise, set<int> &cinds, int ncpts)
{
  double inoise2 = inoise*0.5;

  for (int i=0; i<ncpts; i++)
    {
      // make random point in 0.5m sphere
      Vector4d pt;
      pt[0] = drand48() - 0.5;
      pt[1] = drand48() - 0.5;
      pt[2] = drand48() - 0.5;
      pt[3] = 1.0;
      int pti = sba.addPoint(pt);

      // now add projections to each camera
      set<int>::iterator it;
      for (it=cinds.begin(); it!=cinds.end(); it++)
	{
	  Node &nd = sba.nodes[*it];
	  Vector2d qi;
	  Vector3d qw;
	  qw = nd.w2n * pt; // point in camera coords
          nd.project2im(qi,pt); // point in image coords
	  qi[0] += drand48() * inoise - inoise2; // add noise now
	  qi[1] += drand48() * inoise - inoise2;
	  //              cout << "Cam and pt indices: " << cind << " " << Wptind[i] << endl;
	  sba.addMonoProj(*it, pti, qi);
	}
    }
}

// get a set of camera connections for <ci>
// 1/2 are near neighbors, 1/2 are far (or random)
void
find_cams(SysSBA &sba, int ci, set<int> &cinds, int nconns)
{
  int nnear = nconns/2;
  cinds.insert(ci);
  Vector3d cv = sba.nodes[ci].trans.head(3);
  
  // order cams relative to ci
  map<double,int> nn;
  for (int i=0; i<(int)sba.nodes.size(); i++)
    {
      if (i == ci) continue;
      double dist = cv.dot(sba.nodes[i].trans.head(3));
      nn.insert(pair<double,int>(dist,i));
    }

  // insert near ones
  map<double,int>::iterator it;
  int k = 0;
  for (it=nn.begin(); it!=nn.end(); it++, k++)
    {
      if (k >= nnear) break;
      cinds.insert((*it).second);
    }

  // insert far ones
  k = 0;
  it=nn.begin();
  for (int i=0; i< (int)sba.nodes.size() - nnear - 10; i++)
    it++;
  for (; it!=nn.end(); it++, k++)
    {
      if (k >= nnear) break;
      cinds.insert((*it).second);
    }

#if 0
  // insert far ones
  int ncams = sba.nodes.size();
  for (int i=ncams-20; i<ncams-20+nnear; i++)
    cinds.insert(nn[i]);
#endif
}

void
sphere_setup(SysSBA &sba, CamParams &cpars, vector<Matrix<double,6,1>, Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
	     int ncams,		// number of cameras
	     int ncpts,		// number of new pts per camera
	     int nccs,		// number of camera connections per camera
             double inoise, double pnoise, double qnoise)
{
  // random seed
  unsigned short seed = (unsigned short)utime();
  seed48(&seed);

  //
  // cams around sphere, at radius 1.0
  //

  for (int i=0; i<ncams; i++)
    {
      double x = drand48()*2.0 - 1.0; // in the range [-1.0,1.0]
      double y = drand48()*2.0 - 1.0;
      double z = drand48()*2.0 - 1.0;
      double norm = sqrt(x*x + y*y + z*z);
      Vector4d frt(x/norm,y/norm,z/norm,1.0); // frame position in the world
      

      // frame rotation in the world, pointing towards center
      // could also set a random rotation here
      //      double ang = drand48()*2.0*M_PI; // radians in [0,2pi]
      Quaternion<double> frq;
      frq.setFromTwoVectors(Vector3d(0,0,1),-frt.head(3));
      frq.normalize();
      if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();

      // make node
      Node nd;
      nd.qrot = frq;
      nd.normRot();
      //      cout << "Quaternion: " << nd.qrot.transpose() << endl;
      nd.trans = frt;
      //      cout << "Translation: " << nd.trans.transpose() << endl << endl;
      nd.setTransform();        // set up world2node transform
      nd.setKcam(cpars);        // set up node2image projection
      nd.setDr(sba.useLocalAngles);

      // save it
      Matrix<double,6,1> pos;
      pos.head(3) = frt.head(3);
      pos.segment<3>(3) = frq.coeffs().head(3);
      cps.push_back(pos);

      // add to SBA
      sba.nodes.push_back(nd);

      // print out camera poses
      //      printf("   %f %f %f %f %f %f\n", frt[0],frt[1],frt[2],frq.x(),frq.y(),frq.z());
    }

  // add point projections
  for (int i=0; i<(int)cps.size(); i++)
    {
      // form list of connected cameras
      set<int> cinds;
      find_cams(sba,i,cinds,nccs);
      add_sphere_points(sba, inoise, cinds, ncpts);
    }

  // add error to initial positions
  for (size_t i=0; i<sba.nodes.size(); i++)
    {
      Node &nd = sba.nodes[i];
      if (i>0)          // add error
        {
          Vector4d frt2(drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2,
                        drand48()*pnoise - pnoise/2,
                        0.0);
          nd.trans += frt2;

          Vector4d frq2(drand48()*qnoise - qnoise/2,
                        drand48()*qnoise - qnoise/2,
                        drand48()*qnoise - qnoise/2,
                        0.0);
          nd.qrot.coeffs() += frq2;
          double nrm = nd.qrot.vec().squaredNorm();
          if (nrm > 0.9999)
            nd.qrot.vec() = -nd.qrot.vec() / (sqrt(nrm)*1.0001);
          nd.normRot();

          nd.setTransform();    // set up world2node transform
          nd.setProjection();   // set up node2image projection
          nd.setDr(sba.useLocalAngles); // set rotational derivatives
        }
    }
 }
