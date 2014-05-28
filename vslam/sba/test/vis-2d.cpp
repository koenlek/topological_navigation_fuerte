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
// Visualizing pose graph results for 2d datasets
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

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

// use zero coords for nodes
static int useInit = 0;


void buildConstraintCache(std::multimap<int,int>& constraintCache, std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >& cind){
  for (size_t i=0; i<cind.size(); i++){
    int idx_a=cind[i].x();
    int idx_b=cind[i].y();
    if (idx_a<idx_b){
      int aux=idx_a;
      idx_a=idx_b;
      idx_b=aux;
    }
    constraintCache.insert(make_pair(idx_a,i));
  }
}

//
// add a single node to the graph, in the position given by sequential constraint
//

int 
addnode(SysSPA2d &spa, int n, 
        std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ntrans,
        std::vector< double > &arots,
        std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,
        std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ctrans,
        std::vector< double > &carot,
        std::vector< Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3> > > &cvar,
        std::multimap<int,int>& constraintCache)
{
  Node2d &nd0 = spa.nodes[n-1];
  Node2d nd1;

  int cnum=0;
  
  std::multimap<int,int>::iterator it=constraintCache.lower_bound(n);
  bool first=true;
  while (it!=constraintCache.end() && (cind[it->second].x()<n || cind[it->second].y()<n)){
    int i = it->second;
    if (first) {
      if (cind[i].x() ==  n-1&&
	  cind[i].y() == n)     // found it!
	{
	  // set node rotation
	  double a = carot[i];
	  nd1.arot = nd0.arot + a;
	  nd1.normArot();
	  
	  // set node translation
	  Vector2d itr = nd0.w2n.transpose().block<2,2>(0,0) * ctrans[i];
	  nd1.trans = nd0.trans;
	  nd1.trans.head(2) += itr;
	  
          if (useInit == 1)     // file init
            {
              nd1.trans.head(2) = ntrans[n]; // init to file params
              nd1.arot = arots[n];
              nd1.normArot();
            }
          else if (useInit == 2)
	    nd1.trans.head(2) = Vector2d(0.0,0.001*n);  // init to zero

	  // add in to system
	  nd1.setTransform();   // set up world2node transform
	  nd1.setDr();
	  spa.nodes.push_back(nd1);
          //          cout << "Found node " << n << endl;
	  first=false;
	}
      else if (cind[i].x() == n &&
	       cind[i].y() == n-1) // found it, but reversed
	{
          // set node rotation
          double a = -carot[i];
          nd1.arot = nd0.arot + a;
          nd1.normArot();
          nd1.setTransform();   // set up world2node transform
	
          // set node translation
          Vector2d itr = nd1.w2n.transpose().block<2,2>(0,0) * ctrans[i];
          nd1.trans = nd0.trans;
          nd1.trans.head(2) -= itr;
	
          if (useInit == 1)     // file init
            {
              nd1.trans.head(2) = ntrans[n]; // init to file params
              nd1.arot = arots[n];
              nd1.normArot();
            }
          else if (useInit == 2)
	    nd1.trans.head(2) = Vector2d(0.0,0.001*n);  // init to zero

          // add in to system
          nd1.setDr();
          spa.nodes.push_back(nd1);
          //          cout << "Found node rev " << n << endl;
          first=false;
        }
    }
    Con2dP2 con;
    con.ndr = cind[i].x();
    con.nd1 = cind[i].y();
    
    if ((con.ndr == n && con.nd1 <= n-1) ||
	(con.nd1 == n && con.ndr <= n-1))
      {
	cnum++;
	con.tmean = ctrans[i];
	con.amean = carot[i];
	con.prec = cvar[i];       // ??? should this be inverted ???
	spa.p2cons.push_back(con);
      } 
    it++;
  }
  return cnum;
}




// drawing the graph
// draw the graph on rviz
void
drawgraph(SysSPA2d &spa, ros::Publisher &marker_pub, ros::Publisher &marker2_pub, ros::Publisher &marker_pts_pub)
{
    visualization_msgs::Marker marker, marker2, marker_pts;
    marker.header.frame_id = "/pgraph";
    marker.header.stamp = ros::Time();
    marker.ns = "pgraph";
    marker.id = 0;
//    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker2 = marker;
    marker2.color.r = 1.0f;
    marker2.color.g = 0.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0;

    marker_pts = marker;
    marker_pts.scale.x = 0.2;
    marker_pts.scale.y = 0.2;
    marker_pts.scale.z = 0.01;
    marker_pts.color.r = 1.0f;
    marker_pts.color.g = 1.0f;
    marker_pts.color.b = 0.0f;
    marker_pts.color.a = 1.0;

    int ncons = spa.p2cons.size();
    int ngood = 0;
    int nbad = 0;
    double good_thresh = 0.02;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker2.type = visualization_msgs::Marker::LINE_LIST;
    marker_pts.type = visualization_msgs::Marker::POINTS;

    // find number of good and bad links
    for (int i=0; i<ncons; i++)
      {
        Con2dP2 &con = spa.p2cons[i];
        Node2d &nd0 = spa.nodes[con.ndr];
        Node2d &nd1 = spa.nodes[con.nd1];
        if (con.calcErrDist(nd0,nd1) > good_thresh*good_thresh)
          nbad++;
        else 
          ngood++;
      }

    marker.points.resize(2*ngood);
    marker2.points.resize(2*nbad);
    nbad += nbad-1;
    ngood += ngood-1;

    for (int i=0; i<ncons; i++)
      {
        Con2dP2 &con = spa.p2cons[i];
        Node2d &nd0 = spa.nodes[con.ndr];
        Node2d &nd1 = spa.nodes[con.nd1];
        
        if (con.calcErrDist(nd0,nd1) > good_thresh*good_thresh)
          {
            marker2.points[nbad].x = nd0.trans.x();
            marker2.points[nbad].y = nd0.trans.y();
            marker2.points[nbad--].z = 0.0;
            marker2.points[nbad].x = nd1.trans.x();
            marker2.points[nbad].y = nd1.trans.y();
            marker2.points[nbad--].z = 0.0;
          }
        else
          {
            marker.points[ngood].x = nd0.trans.x();
            marker.points[ngood].y= nd0.trans.y();
            marker.points[ngood--].z = 0.0;
            marker.points[ngood].x = nd1.trans.x();
            marker.points[ngood].y = nd1.trans.y();
            marker.points[ngood--].z = 0.0;
          }
      }
    
    // find number of points
    int nskip = 20;
    int nscans = spa.scans.size();
    if (nscans > (int)spa.nodes.size())
      nscans = spa.nodes.size();
    int npts = 0;
    for (int i=0; i<nscans; i++)
      npts += spa.scans[i].size()/nskip + 1;
    marker_pts.points.resize(npts);

    // draw points
    for (int i=0; i<nscans; i++)
      {
        std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &scan = spa.scans[i];
        Node2d &nd = spa.nodes[i];
        Matrix<double,2,3> n2w;
        n2w.col(2) = nd.trans.head(2);
        n2w.block<2,2>(0,0) = nd.w2n.block<2,2>(0,0).transpose();
        for (int j=0; j<(int)scan.size(); j+=nskip)
          {
            Vector3d v;
            v.head(2) = scan[j];
            v(2) = 1.0;
            Vector2d p = n2w * v;
            marker_pts.points[--npts].x = p[0];
            marker_pts.points[npts].y = p[1];
            marker_pts.points[npts].z = 0.0;
          }
      }

    marker_pub.publish(marker);
    marker2_pub.publish(marker2);
    marker_pts_pub.publish(marker_pts);
}  



//
// first argument is the name of input file.
// files are in Freiburg's VERTEX2/EDGE2/POINT2 format
// runs SPA
//

int main(int argc, char **argv)
{
  char *fin;

  if (argc < 2)
    {
      cout << "Arguments are:  <input filename> [<number of nodes to use>] [<iters (5)>] [<1=file init, 2=zero init>]" << endl;
      return -1;
    }

  // number of nodes to increment each time
  int inn = 10;

  if (argc > 2)
    inn = atoi(argv[2]);

  int doiters = 5;
  if (argc > 3)
    doiters = atoi(argv[3]);

  if (argc > 4)
    useInit = atoi(argv[4]);


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
  int npts = 0;
  for (int i=0; i<(int)scans.size(); i++)
    npts += scans[i].size();

  cout << "[ReadSPA2dFile] Found " << (int)ntrans.size() << " nodes, " 
       << (int)cind.size() << " constraints, and " 
       << npts << " points" << endl;

  cout  << "# building constraint cache... ";
  std::multimap<int,int> constraintCache;
  buildConstraintCache(constraintCache,cind);
  cout  << "done" << endl;

  // system
  SysSPA2d spa;
  spa.scans = scans;

  // use max nodes if we haven't specified it
  int nnodes = ntrans.size();

  // add first node
  Node2d nd;

  // rotation
  nd.arot = arots[0];
  // translation
  Vector3d v;
  v.head(2) = ntrans[0];
  v(2) = 1.0;
  nd.trans = v;

  cout << nd.trans.transpose() << endl << endl;

  // add to system
  nd.setTransform();            // set up world2node transform
  nd.setDr();
  spa.nodes.push_back(nd);
  spa.nFixed = 1;               // one fixed frame

  // set up markers for visualization
  ros::init(argc, argv, "vis_2d");
  ros::NodeHandle n ("~");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("good_graph_links", 0);
  ros::Publisher marker2_pub = n.advertise<visualization_msgs::Marker>("bad_graph_links", 0);
  ros::Publisher marker_pts_pub = n.advertise<visualization_msgs::Marker>("scan_points", 0);

  // wait for the user to signal go-ahead
  cout << "[TestSPA] Ready!" << endl;
  string input = "";
  getline(cin,input);

  int iter = 0;
  double cumTime = 0.0;
  bool contin = false;
  int onn = 1;
  int nn = 1;

  while (ros::ok())
  {
    nn += inn;
    if (nn > nnodes) nn = nnodes;

    // add in nodes
    if (onn < nnodes)
      {
	for (int i=onn; i<nn; i++)
	  addnode(spa, i, ntrans, arots, cind, ctrans, carot, cvar, constraintCache);
      }

    cout << "[SysSPA2d] Using " << (int)spa.nodes.size() << " nodes and " 
         << (int)spa.p2cons.size() << " constraints" << endl;

    if (iter == 0)
      {
        printf("[TestSPA] Distance cost:   %0.3f m rms\n", sqrt(spa.calcCost(true)/(double)spa.p2cons.size()));
        printf("[TestSPA] Chi-square cost: %0.3f\n", spa.calcCost());
      }

    cout << "[TestSPA] Publishing " << iter << endl;
    iter++;
    drawgraph(spa,marker_pub,marker2_pub,marker_pts_pub);

    if (!contin)
      {
        getline(cin,input);
        if (input[0] == 'c')
          {
            contin = true;
            spa.verbose = false;
          }
      }

    // do 10 iterations "t"
    if (input[0] == 't')
      spa.doSPA(40,1.0e-4,1);   
    else
      {
        long long t0, t1;
        t0 = utime();

//        spa.doSPAwindowed(110,10,1.0e-4,1);

        spa.nFixed = 1;         // one fixed frame
	spa.doSPA(doiters,0.0,1);
        //	spa.doDSIF(onn);	// this runs the Delayed Sparse Info Filter

        t1 = utime();
#if 0
        cerr << "#DSIF" << endl;
        double dt=1e-6*(double)(t1-t0) ;
        cumTime+=dt;
        cerr << "nodes= " << spa.nodes.size() 
             << "\t edges= " << spa.p2cons.size()
             << "\t chi2= ?? " 
             << "\t time= " << dt
             << "\t iterations= " << 1
             <<  "\t cumTime= " << cumTime 
             << "\t chi2= " << spa.calcCost() 
             << endl;
#endif
        printf("[TestSPA] Compute took %0.2f ms/iter\n", 0.001*(double)(t1-t0)/(double)doiters);
        printf("[TestSPA] Distance cost:   %0.3f m rms\n", sqrt(spa.calcCost(true)/(double)spa.p2cons.size()));
        printf("[TestSPA] Chi-square cost: %0.3f\n", spa.calcCost());
      }

	
    onn = nn;

    if (nn == nnodes)
      {
        contin = false;
        spa.verbose = true;
      }

    //    if (contin)
    //      ros::Duration(0.01).sleep();
  }

  return 0;
}
