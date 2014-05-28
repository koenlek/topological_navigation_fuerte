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

// Run an Intel node/point file (in TORO format) through SBA

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "sba/read_spa.h"
#include "sba/sba.h"
#include <sba/sba_file_io.h>
#include "frame_common/frame.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <sys/time.h>

using namespace std;
using namespace Eigen;
using namespace sba;
using namespace frame_common;



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

#define SAVE_RESULTS


// draw the graph on rviz
void
drawgraph(SysSBA &sba, ros::Publisher &cam_pub, ros::Publisher &pt_pub, int dec)
{
  visualization_msgs::Marker cammark, ptmark;
  cammark.header.frame_id = "/pgraph";
  cammark.header.stamp = ros::Time();
  cammark.ns = "pgraph";
  cammark.id = 0;
  cammark.action = visualization_msgs::Marker::ADD;
  cammark.pose.position.x = 0;
  cammark.pose.position.y = 0;
  cammark.pose.position.z = 0;
  cammark.pose.orientation.x = 0.0;
  cammark.pose.orientation.y = 0.0;
  cammark.pose.orientation.z = 0.0;
  cammark.pose.orientation.w = 1.0;
  cammark.scale.x = 0.02;
  cammark.scale.y = 0.02;
  cammark.scale.z = 0.02;
  cammark.color.r = 0.0f;
  cammark.color.g = 1.0f;
  cammark.color.b = 1.0f;
  cammark.color.a = 1.0f;
  cammark.lifetime = ros::Duration();
  cammark.type = visualization_msgs::Marker::LINE_LIST;

  ptmark = cammark;
  ptmark.color.r = 1.0f;
  ptmark.color.g = 0.0f;
  ptmark.color.b = 0.0f;
  ptmark.color.a = 0.5f;
  ptmark.scale.x = 0.01;
  ptmark.scale.y = 0.01;
  ptmark.scale.z = 0.01;
  ptmark.type = visualization_msgs::Marker::POINTS;


  // draw points, decimated
  int npts = sba.tracks.size();
  ptmark.points.resize(npts/dec+1);
  for (int i=0, ii=0; i<npts; i+=dec, ii++)
    {
      Vector4d &pt = sba.tracks[i].point;
      ptmark.points[ii].x = pt(0);
      ptmark.points[ii].y = pt(2);
      ptmark.points[ii].z = -pt(1);
    }

  // draw cameras
  int ncams = sba.nodes.size();
  cammark.points.resize(ncams*6);
  for (int i=0, ii=0; i<ncams; i++)
    {
      Node &nd = sba.nodes[i];
      Vector3d opt;
      Matrix<double,3,4> tr;
      transformF2W(tr,nd.trans,Quaternion<double>(nd.qrot));

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0,0.3,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0.1,0,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0.1,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = -opt.y();
    }
  
  cam_pub.publish(cammark);
  pt_pub.publish(ptmark);
}



//
// first argument is the name of input file, Bundler format
//    expects good focal length and distortion correction
// runs sba
//

int main(int argc, char **argv)
{
  char *fin;

  if (argc < 3)
    {
      cout << "Arguments are: <cam params> <input filename> [<number of nodes to use>]" << endl;
      return -1;
    }

  // number of nodes to use
  int nnodes = 1000000;

  if (argc > 3)
    nnodes = atoi(argv[3]);

  int doiters = 10;
  if (argc > 4)
    doiters = atoi(argv[4]);

  fin = argv[2];

  // get camera parameters, in the form: fx fy cx cy tx
  fstream fstr;
  fstr.open(argv[1],fstream::in);
  if (!fstr.is_open())
    {
      printf("Can't open camera file %s\n",argv[1]);
      exit(0);
    }
  CamParams camp;
  fstr >> camp.fx;
  fstr >> camp.fy;
  fstr >> camp.cx;
  fstr >> camp.cy;
  fstr >> camp.tx;

  cout << "Cam params: " << camp.fx << " " << camp.fy << " " << camp.cx
       << " " << camp.cy << " " << camp.tx << endl;


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

  cout << "[ReadSPAFile] Found " << (int)ntrans.size() << " nodes and " 
       << (int)cind.size() << " constraints, " << (int)tracks.size()
       << " projection pairs" << endl;


  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  ros::NodeHandle n ("~");
  ros::Publisher pt_pub = n.advertise<visualization_msgs::Marker>("points", 0);
  ros::Publisher cam_pub = n.advertise<visualization_msgs::Marker>("cameras", 0);

  // construct an SBA system
  SysSBA sba;
  Node::initDr();

  vector<Frame> frames;
  Frame frm;
  frm.setCamParams(camp);       // put in camera params

  // add in nodes
  for (int i=0; i<(int)ntrans.size(); i++)
    {
      if (i>=nnodes) break;

      Node nd;

      // rotation
      Quaternion<double> frq;
      frq.coeffs() = nqrot[i];
      frq.normalize();
      if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
      nd.qrot = frq.coeffs();

      // translation
      Vector4d v;
      v.head(3) = ntrans[i];
      v(3) = 1.0;
      nd.trans = v;
      nd.setTransform();        // set up world2node transform
      nd.setKcam(camp);         // set up node2image projection
      nd.setDri();              // set rotational derivatives

      // add to system
      sba.nodes.push_back(nd);

      // set up frame
      frames.push_back(frm);
    }


  // add in points and projections
  // first version, just do 2-point tracks
  std::vector<std::pair<int,int> > pairs;
  double ftx = camp.fx * camp.tx;
  for (int i=0; i<(int)tracks.size(); i++)
    {
      tinfo &ti = tracks[i];
      if (ti.fn0 >= (int)frames.size() || ti.fn1 >= (int)frames.size())
        {
          cout << "Bad frame index at track " << i << endl;
          return -1;
        }
      Frame &f0 = frames[ti.fn0];
      Frame &f1 = frames[ti.fn1];
      Node &nd0 = sba.nodes[ti.fn0];
      Node &nd1 = sba.nodes[ti.fn1];
      int fpn0 = ti.fpn0;
      int fpn1 = ti.fpn1;
      if (fpn0 >= (int)f0.kpts.size()) // new frame point
        {
          f0.kpts.resize(fpn0+1);
          f0.ipts.resize(fpn0+1,-1);
          f0.pts.resize(fpn0+1);
        }
      if (fpn1 >= (int)f1.kpts.size()) // new frame point
        {
          f1.kpts.resize(fpn1+1);
          f1.ipts.resize(fpn1+1,-1);
          f1.pts.resize(fpn1+1);
        }

      // set up point identity pairs
      int pn0 = f0.ipts[fpn0];
      int pn1 = f1.ipts[fpn1];
      if (pn0 < 0)              // new point
        f0.ipts[fpn0] = i;
      else
        pairs.push_back(std::pair<int,int>(pn0,i));
      if (pn1 < 0)              // new point
        f1.ipts[fpn1] = i;
      else
        pairs.push_back(std::pair<int,int>(pn1,i));


      // just add a new track for two frames
      Point pt0(ti.x0,ti.y0,ti.z0,1.0);
      Matrix<double,3,4> f2w;
      Quaterniond qr;
      qr = nd0.qrot;
      transformF2W(f2w,nd0.trans,qr);
      pt0.head(3) = f2w*pt0;
      sba.addPoint(pt0);
      Vector2d kp0,kp1;
      nd0.project2im(kp0,pt0);

      Point pt1(ti.x1,ti.y1,ti.z1,1.0);
      qr = nd1.qrot;
      transformF2W(f2w,nd1.trans,qr);
      pt1.head(3) = f2w*pt1;
      nd1.project2im(kp1,pt1);

#if 0
      cout << kp0.transpose() << endl;
      cout << ti.u0 << " " << ti.v0 << " " << ti.u0-ftx/ti.z0 << endl;
      cout << kp1.transpose() << endl;
      cout << ti.u1 << " " << ti.v1 << " " << ti.u1-ftx/ti.z1 << endl;
      cout << pt0.head(3).transpose() << endl;
      cout << pt1.head(3).transpose() << endl << endl;
#endif

      // projections
      int pti = sba.tracks.size() - 1;
      Vector3d nkp0(ti.u0,ti.v0,ti.u0-ftx/ti.z0);
      sba.addStereoProj(ti.fn0,pti,nkp0);
      Vector3d nkp1(ti.u1,ti.v1,ti.u1-ftx/ti.z1);
      sba.addStereoProj(ti.fn1,pti,nkp1);
    }
  
  
  cout << "Number of tracks: " << sba.tracks.size() << endl;
  cout << "Initial RMS cost per projection: " << sqrt(sba.calcCost()/(double)(sba.tracks.size()*2)) << endl;
  sba.printStats();

  // draw and wait
  cout << endl << "drawing..." << endl << endl;
  drawgraph(sba,cam_pub,pt_pub,2); // every 2nd point
  char buf[10];
  cin.getline(buf,10);

  long long t0,t1;

#if 0
  t0 = utime();
  sba.doSBA(10,1.0e-4,1);
  t1 = utime();
  cout << endl << "drawing..." << endl << endl;
  drawgraph(sba,cam_pub,pt_pub,2); // every 2nd point
  cin.getline(buf,10);
#endif

  // merge point tracks
  cout << "Found " << pairs.size() << " connected tracks" << endl;
#if 0
  for (int i=0; i<(int)pairs.size(); i++)
    cout << pairs[i].first << " " << pairs[i].second << endl;
#endif
  sba.mergeTracks(pairs);
  cout << "Number of tracks: " << sba.tracks.size() << endl;
  cout << "Initial RMS cost per projection: " << sqrt(sba.calcCost()/(double)(sba.tracks.size()*2)) << endl;
  sba.printStats();


#if 1
  sba::writeLourakisFile((char *)"intel", sba);
  cout << endl << "Wrote SBA system in Lourakis format" << endl << endl;
#endif

  // file for saving results
#ifdef SAVE_RESULTS
  FILE *fd = fopen("results.txt","a");
#endif

      // average track size
      int npts = sba.tracks.size();
      double tsize = 0;             // average track size
      for (int i=0; i<npts; i++)
        tsize += sba.tracks[i].projections.size();
      tsize = tsize/(double)npts;

      // matrix connections
      int ncams = sba.nodes.size();
      int nprjs = 0;
      MatrixXi conns;
      conns.setIdentity(ncams,ncams);
      for (int i=0; i<(int)sba.tracks.size(); i++)
        {
          ProjMap &prjs = sba.tracks[i].projections;
          nprjs += prjs.size();
          if (prjs.size()>1)
            for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
              {
                Proj &prj = itr->second;
                for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                  {
                    Proj &prj2 = itr2->second;
                    conns(prj.ndi,prj2.ndi) = 1;
                    conns(prj2.ndi,prj.ndi) = 1;
                  }
              }
        }

      int tot = 0;
      for (int i=0; i<ncams; i++)
        for (int j=0; j<ncams; j++)
          tot += conns(i,j);
      double mfill = (double)tot/(double)(ncams*ncams);

      printf("\nCams: %d  Avg projs: %0.1f  Avg. track size: %0.1f  Percent fill: %0.1f\n\n",
             ncams, (double)nprjs/(double)ncams, tsize, 100.0*mfill);

#ifdef SAVE_RESULTS
      fprintf(fd,"%d %0.1f %0.1f %0.1f ",
              ncams, (double)nprjs/(double)ncams, tsize, 100.0*mfill);
#endif

  sba.nFixed = 1;
  sba.csp.useCholmod = true;

  t0 = utime();
  sba.doSBA(1,1.0e-4,1);
  t1 = utime();

#ifdef SAVE_RESULTS
      fprintf(fd,"%0.1f %0.1f %0.1f %0.1f\n",
              (sba.t2-sba.t1)*0.001,
              (sba.t1-sba.t0)*0.001,
              (sba.t3-sba.t2)*0.001,
              (t1-t0)*0.001);
      fflush(fd);
  fclose(fd);
#endif


  cout << endl << "drawing..." << endl << endl;
  drawgraph(sba,cam_pub,pt_pub,2); // every 2nd point
  cin.getline(buf,10);

  // save nodes and points to a file
  char *name = "intel-sys.txt";
  FILE *fn = fopen(name,"w");
  if (fn == NULL)
    {
      cout << "[WriteFile] Can't open file " << name << endl;
      return -1;
    }
  fprintf(fn,"Node poses: node #, trans XYZ, rot XYZW\n");
  for (int i=0; i<(int)sba.nodes.size(); i++)
    {
      Node nd = sba.nodes[i];
      fprintf(fn,"%d %f %f %f %f %f %f %f\n", i, nd.trans(0), nd.trans(1), nd.trans(2),
              nd.qrot.x(), nd.qrot.y(), nd.qrot.z(), nd.qrot.w());
    }
  cout << "Wrote node poses to file " << name << endl;
  

  return 0;

#if 0
  cout << endl;

  cout << "Bad projs (> 100 pix): " << sba.countBad(100.0) 
       << "  Cost without: " << sqrt(sba.calcCost(100.0)/nprjs) << endl;
  cout << "Bad projs (> 20 pix): " << sba.countBad(20.0)
       << "  Cost without: " << sqrt(sba.calcCost(20.0)/nprjs) << endl;
  cout << "Bad projs (> 10 pix): " << sba.countBad(10.0)
       << "  Cost without: " << sqrt(sba.calcCost(10.0)/nprjs) << endl;
  int n = sba.removeBad(20.0);
  cout << "Removed " << n << " projs with >10px error" << endl;
  sba.printStats();
#endif

#if 0
  sba.doSBA(2);
  sba.setupSys(0.0);
  sba.writeSparseA((char *)"A819.venice");
#endif

  //  sba.setConnMat(minpts);
  //  sba.setConnMatReduced(minpts);             // finds spanning tree

  //  sba.doSBA(30,1.0e-3,true);
  sba.doSBA(1,1e-3,1);
  drawgraph(sba,cam_pub,pt_pub,2);
  for (int i=0; i<20; i++)
    {
      sba.doSBA(1,0.0,1);      
      drawgraph(sba,cam_pub,pt_pub,2);
    }

  return 0;
}
