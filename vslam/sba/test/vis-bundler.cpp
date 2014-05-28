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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

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
      ptmark.points[ii].z = pt(1);
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
      cammark.points[ii++].z = nd.trans.y();
      opt = tr*Vector4d(0,0,0.3,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = nd.trans.y();
      opt = tr*Vector4d(0.1,0,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = opt.y();

      cammark.points[ii].x = nd.trans.x();
      cammark.points[ii].y = nd.trans.z();
      cammark.points[ii++].z = nd.trans.y();
      opt = tr*Vector4d(0,0.1,0,1);
      cammark.points[ii].x = opt.x();
      cammark.points[ii].y = opt.z();
      cammark.points[ii++].z = opt.y();
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

  if (argc < 2)
    {
      cout << "Arguments are:  <input filename> [<min conn pts>]" << endl;
      return -1;
    }

  int minpts = 0;
  if (argc > 2)
    minpts = atoi(argv[2]);

  fin = argv[1];

  vector< Vector3d, Eigen::aligned_allocator<Vector3d> > camps;	// cam params <f d1 d2>
  vector< Matrix3d, Eigen::aligned_allocator<Matrix3d> > camRs;	// cam rotation matrix
  vector< Vector3d, Eigen::aligned_allocator<Vector3d> > camts;	// cam translation
  vector< Vector3d, Eigen::aligned_allocator<Vector3d> > ptps;	// point position
  vector< Vector3i, Eigen::aligned_allocator<Vector3i> > ptcs;	// point color
  vector< vector< Vector4d, Eigen::aligned_allocator<Vector4d> > > ptts; // point tracks - each vector is <camera_index kp_idex u v>

  int ret = ParseBundlerFile(fin, camps, camRs, camts, ptps, ptcs, ptts);
  if (ret < 0)
    return -1;
  int ncams = camps.size();
  int npts  = ptps.size();
  int nprjs = 0;
  for (int i=0; i<npts; i++)
    nprjs += (int)ptts[i].size();
  cout << "Points: " << npts << "  Tracks: " << ptts.size() 
       << "  Projections: " << nprjs << endl;


  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  ros::NodeHandle n ("~");
  ros::Publisher pt_pub = n.advertise<visualization_msgs::Marker>("points", 0);
  ros::Publisher cam_pub = n.advertise<visualization_msgs::Marker>("cameras", 0);

  // construct an SBA system
  SysSBA sys;
  Node::initDr();

  // set up nodes/frames
  cout << "Setting up nodes..." << flush;
  for (int i=0; i<ncams; i++)
    {
      // camera params
      Vector3d &camp = camps[i];
      CamParams cpars = {camp[0],camp[0],0,0,0}; // set focal length, no offset

      //
      // NOTE: Bundler camera coords are rotated 180 deg around the X axis of
      //  the camera, so Z points opposite the camera viewing ray (OpenGL).
      // Note quite sure, but I think this gives the camera pose as
      //  [-R' -R'*t]

      // rotation matrix
      Matrix3d m180x;		// rotate 180 deg around X axis, to convert Bundler frames to SBA frames
      m180x << 1, 0, 0, 0, -1, 0, 0, 0, -1;
      Matrix3d camR = m180x * camRs[i]; // rotation matrix
      Quaternion<double> frq(camR.transpose());	// camera frame rotation, from Bundler docs
      if (frq.w() < 0.0)	// w negative, change to positive
	{
	  frq.x() = -frq.x();
	  frq.y() = -frq.y();
	  frq.z() = -frq.z();
	  frq.w() = -frq.w();
	}

      // translation
      Vector3d &camt = camts[i];
      Vector4d frt;
      frt.head<3>() = -camRs[i].transpose() * camt; // camera frame translation, from Bundler docs
      frt[3] = 1.0;

      Node nd;
      nd.qrot = frq.coeffs();	
      nd.normRot();
      //      cout << "Quaternion: " << nd.qrot.transpose() << endl;
      nd.trans = frt;
      //      cout << "Translation: " << nd.trans.transpose() << endl << endl;
      nd.setTransform();	// set up world2node transform
      nd.setKcam(cpars);	// set up node2image projection
      nd.setDr(true);		// set rotational derivatives
      sys.nodes.push_back(nd);
    }
  cout << "done" << endl;

  // set up points
  cout << "Setting up points..." << flush;
  for (int i=0; i<npts; i++)
    {
      // point
      Vector3d &ptp = ptps[i];
      Point pt;
      pt.head<3>() = ptp;
      pt[3] = 1.0;
      sys.addPoint(pt);
    }
  cout << "done" << endl;


  sys.useLocalAngles = true;    // use local angles
  sys.nFixed = 1;

  // set up projections
  int ntot = 0;
  cout << "Setting up projections..." << flush;
  for (int i=0; i<npts; i++)
    {
      // track
      vector<Vector4d, Eigen::aligned_allocator<Vector4d> > &ptt = ptts[i];
      int nprjs = ptt.size();
      for (int j=0; j<nprjs; j++)
	{
	  // projection
	  Vector4d &prj = ptt[j];
	  int cami = (int)prj[0];
	  Vector2d pt = prj.segment<2>(2);
	  pt[1] = -pt[1];	// NOTE: Bundler image Y is reversed
	  if (cami >= ncams)
	    cout << "*** Cam index exceeds bounds: " << cami << endl;
	  sys.addMonoProj(cami,i,pt); // camera indices aren't ordered
	  ntot++;

#if 0
	  if (ntot==1000000)
	    {
	      Node &nd = sys.nodes[cami];
	      Point &npt = sys.points[i];
	      cout << pt.transpose() << endl;
	      Vector3d pti = nd.w2i * npt;
	      pti = pti / pti[2];
	      cout << pti.transpose() << endl;
	      cout << nd.trans.transpose() << endl;
	      cout << nd.qrot.transpose() << endl;
	    }
#endif

	  //	  if ((ntot % 100000) == 0)
	  //	    cout << ntot << endl;
	}
    }
  cout << "done" << endl;


  if (minpts > 0)
    {
      int nrem = sys.reduceLongTracks(minpts); // tracks greater than minpts size are removed
    //      sys.remExcessTracks(minpts);
      cout << "Split " << nrem << " / " << sys.tracks.size() << " tracks" << endl; 
    }

  double cost = sys.calcCost();
  cout << "Initial squared cost: " << cost << ",  which is " << sqrt(cost/nprjs) << " rms pixels per projection"  << endl;

  sys.nFixed = 1;
  sys.printStats();
  sys.csp.useCholmod = true;


  // draw graph
  cout << endl << "drawing..." << endl << endl;
  drawgraph(sys,cam_pub,pt_pub,2); // every 2nd point


#if 0
  sys.writeFile((char *)"sbasys");
  cout << endl << "Wrote SBA system in Lourakis format" << endl << endl;
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

  //  sys.doSBA(30,1.0e-3,true);
  sys.doSBA(1,1e-3,1);
  drawgraph(sys,cam_pub,pt_pub,2);
  for (int i=0; i<20; i++)
    {
      sys.doSBA(1,0.0,1);      
      drawgraph(sys,cam_pub,pt_pub,2);
    }

  cout << endl << "Switch to full system" << endl;
  sys.connMat.resize(0);


  // reset projections here
  // just use old points
  sys.tracks.resize(npts);

  // set up projections
  sys.tracks.resize(0);
  cout << "Setting up projections..." << flush;
  for (int i=0; i<npts; i++)
    {
      // track
      vector<Vector4d, Eigen::aligned_allocator<Vector4d> > &ptt = ptts[i];
      int nprjs = ptt.size();
      for (int j=0; j<nprjs; j++)
	    {
	      // projection
	      Vector4d &prj = ptt[j];
	      int cami = (int)prj[0];
	      Vector2d pt = prj.segment<2>(2);
	      pt[1] = -pt[1];	// NOTE: Bundler image Y is reversed
	      if (cami >= ncams)
	        cout << "*** Cam index exceeds bounds: " << cami << endl;
	      sys.addMonoProj(cami,i,pt); // camera indices aren't ordered
	      ntot++;
	    }
    }
  cout << "done" << endl;


  sys.doSBA(20,1e-3,1);
  drawgraph(sys,cam_pub,pt_pub,2);

  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) 
       << "  Cost without: " << sqrt(sys.calcCost(10.0)/nprjs) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0)
       << "  Cost without: " << sqrt(sys.calcCost( 5.0)/nprjs) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0)
       << "  Cost without: " << sqrt(sys.calcCost( 2.0)/nprjs) << endl << endl;

  sys.removeBad(10.0);
  cout << "Removed projs with >10px error" << endl;

  sys.doSBA(10,1e-3,true);
  cout << "Bad projs (> 10 pix): " << sys.countBad(10.0) << endl;
  cout << "Bad projs (>  5 pix): " << sys.countBad( 5.0) << endl;
  cout << "Bad projs (>  2 pix): " << sys.countBad( 2.0) << endl << endl;

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
