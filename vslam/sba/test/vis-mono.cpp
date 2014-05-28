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

#include "sba/sba_setup.h"

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
drawgraph(SysSPA &spa, ros::Publisher &cam_pub, ros::Publisher &link_pub)
{
  visualization_msgs::Marker cammark, linkmark;
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

  linkmark = cammark;
  linkmark.color.r = 1.0f;
  linkmark.color.g = 1.0f;
  linkmark.color.b = 0.0f;
  linkmark.color.a = 1.0f;


  // draw cameras
  int ncams = spa.nodes.size();
  cammark.points.resize(ncams*6);
  printf("Cams: %d\n", ncams);
  for (int i=0, ii=0; i<ncams; i++)
    {
      Node &nd = spa.nodes[i];
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
  
  // draw links
  int nlinks = spa.p2cons.size();
  linkmark.points.resize(2*nlinks);
  for (int i=0; i<nlinks; i++)
    {
      ConP2 &con = spa.p2cons[i];
      Node &nd0 = spa.nodes[con.ndr];
      Node &nd1 = spa.nodes[con.nd1];
        
      linkmark.points[i*2].x = nd0.trans.x();
      linkmark.points[i*2].y = nd0.trans.z();
      linkmark.points[i*2].z = nd0.trans.y();
      linkmark.points[i*2+1].x = nd1.trans.x();
      linkmark.points[i*2+1].y = nd1.trans.z();
      linkmark.points[i*2+1].z = nd1.trans.y();
    }


  link_pub.publish(linkmark);
  cam_pub.publish(cammark);

}


//
// setup of precision matrices
//

static Eigen::Matrix<double,6,6> n2prec, n2vprec, n2aprec, n2bprec, diagprec;


void initPrecs()
{
  // set up some precision matrices
  n2prec << 260312.1329351594, 17615.81091248868, -11716.3738046826,
           -260221.3577238563, 3028947.570775249, 284048.6838048229,
    17615.81091248783, 369156.349498884, -8122.584888439054,
    -4130281.103526653, 265383.1196958761, 523737.7444220608,
    -11716.3738046842, -8122.58488844048, 673.3469031685361,
    93635.22686723019, -137533.0434459766, -22834.5012408561,
    -260221.3577238639, -4130281.103526646, 93635.22686720481,
    52493931.52684124, -4078689.933502881, -9475682.025736494,
    3028947.570775286, 265383.119695912, -137533.0434459558,
    -4078689.933502988, 39416288.19312727, 3894322.443643413,
    284048.6838048277, 523737.7444220638, -22834.50124085596,
    -9475682.025736755, 3894322.443643621, 50690679.29036696;

  n2vprec << 624875.2423563644,-8.596260869004408e-11,10576.14746839753,
    -65704.86829639168,10646337.23355757,646569.8439109828,
    -1.045228848835824e-10,-2.955857780762017e-10,9.820269042393193e-10,
    6.912159733474255e-09,-3.751665644813329e-09,-3.511559043545276e-08,
    10576.14746839765,7.860307960072532e-10,224224.9112157905,
    -233966.3120641535,77714.35666432518,65704.86829639639,
    -65704.86829639156,8.021743269637227e-09,-233966.312064158,
    7256072.962556601,-1242408.349188809,197719.0360238712,
    10646337.23355758,-6.682398634438869e-09,77714.35666432098,
    -1242408.349188721,214456943.0273151,11161674.13523376,
    646569.8439109783,-3.356490196892992e-08,65704.86829639817,
    197719.0360238167,11161674.13523367, 19698666.98402661;

  n2aprec << 229528.3846633453, 886.7480854882738, -10039.08940223746, 62445.98594207098, 2715273.460194867, 106542.6230004076,
    886.7480854885912, 319242.7032811134, -6397.916315207351, -3608430.146373766, -49269.13482550202, 582748.417531022,
    -10039.08940223649, -6397.916315208951, 565.7603057193751, 69152.18264815415, -117569.9760459389, -16259.89068069827,
    62445.98594206382, -3608430.146373736, 69152.1826481162, 47244836.25653829, 1303537.745687656, -9808843.224988466,
    2715273.46019485, -49269.13482549335, -117569.9760459207, 1303537.745687651, 35830355.245529, 709155.852370202,
    106542.623000413, 582748.4175310251, -16259.89068069991, -9808843.224988459, 709155.8523703497, 48304469.04982638;

  n2bprec << 148324.039595044, 222.4623044457281, -19531.70697504873, -10192.06466578297, 1631677.485087357, 60190.82294241861,
    222.4623044456828, 200041.4398061978, -4054.812572933995, -2258670.079144401, 29578.86052762273, 799843.0721628161,
    -19531.70697504886, -4054.812572933865, 2652.99484259674, 46794.05582115334, -215409.6450292048, -24019.87801347017,
    -10192.06466578462, -2258670.079144401, 46794.05582115659, 28945336.2353294, -434508.6610355716, -12934377.41525949,
    1631677.485087361, 29578.86052762576, -215409.6450292043, -434508.6610355551, 20018126.98420228, 1153711.950184977,
    60190.82294241752, 799843.0721628153, -24019.8780134693, -12934377.41525948, 1153711.950184968, 22955884.81085673;


#if 0
  // this has zeros for rot-trans interaction
  n2prec << 260312.1329351594, 17615.81091248868, -11716.3738046826, 0.0, 0.0, 0.0,
    17615.81091248783, 369156.349498884, -8122.584888439054, 0.0, 0.0, 0.0,
    -11716.3738046842, -8122.58488844048, 673.3469031685361, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 52493931.52684124, -4078689.933502881, -9475682.025736494,
    0.0, 0.0, 0.0, -4078689.933502988, 39416288.19312727, 3894322.443643413,
    0.0, 0.0, 0.0, -9475682.025736755, 3894322.443643621, 50690679.29036696;

  n2vprec << 624875.2423563644,-8.596260869004408e-11,10576.14746839753,0,0,0,
    -1.045228848835824e-10,-2.955857780762017e-10,9.820269042393193e-10,0,0,0,
    10576.14746839765,7.860307960072532e-10,224224.9112157905,0,0,0,
    0,0,0, 7256072.962556601,-1242408.349188809,197719.0360238712,
    0,0,0, -1242408.349188721,214456943.0273151,11161674.13523376,
    0,0,0,  197719.0360238167,11161674.13523367, 19698666.98402661;

  n2aprec << 229528.3846633453, 886.7480854882738, -10039.08940223746, 0,0,0,
    886.7480854885912, 319242.7032811134, -6397.916315207351,  0,0,0,
    -10039.08940223649, -6397.916315208951, 565.7603057193751,  0,0,0,
    0,0,0, 47244836.25653829, 1303537.745687656, -9808843.224988466,
    0,0,0, 1303537.745687651, 35830355.245529, 709155.852370202,
    0,0,0, -9808843.224988459, 709155.8523703497, 48304469.04982638;

  n2bprec << 148324.039595044, 222.4623044457281, -19531.70697504873, 0,0,0,
    222.4623044456828, 200041.4398061978, -4054.812572933995, 0,0,0,
    -19531.70697504886, -4054.812572933865, 2652.99484259674, 0,0,0,
    0,0,0, 28945336.2353294, -434508.6610355716, -12934377.41525949,
    0,0,0, -434508.6610355551, 20018126.98420228, 1153711.950184977,
    0,0,0, -12934377.41525948, 1153711.950184968, 22955884.81085673;
#endif

#if 1
  n2prec  *= 1.0/100000.0;
  n2vprec *= 1.0/100000.0;
  n2aprec *= 1.0/100000.0;
  n2bprec *= 1.0/100000.0;
#endif

  diagprec.setIdentity();
  diagprec = diagprec*(1000);  
  diagprec.diagonal().head(3) *= .0001;
}



//
// first argument is the name of input file, Bundler format
//    expects good focal length and distortion correction
// runs sba
//

int main(int argc, char **argv)
{
  // args
  double lscale = 0.0;
  double con_weight = 1.0;

  printf("Args: <lscale 0.0>  <scale weight 1.0>\n");

  if (argc > 1)
    lscale = atof(argv[1]);

  if (argc > 2)
    con_weight = atof(argv[2]);


  // set up markers for visualization
  ros::init(argc, argv, "VisBundler");
  ros::NodeHandle n ("~");
  ros::Publisher link_pub = n.advertise<visualization_msgs::Marker>("links", 0);
  ros::Publisher cam_pub = n.advertise<visualization_msgs::Marker>("cameras", 0);

  // set up system
  SysSPA spa;
  Node::initDr();               // set up fixed jacobians
  initPrecs();

  vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > cps;
  double kfang = 5.0;
  double kfrad = kfang*M_PI/180.0;

  // create a spiral test trajectory
  // connections are made between a frame and its three successors

  spa.nFixed = 1;               // three fixed frames
  spa_spiral_setup(spa, true, cps, // use cross links
#if 1
                   n2prec, n2vprec, n2aprec, n2bprec,  // rank-deficient
#else
                   diagprec, diagprec, diagprec, diagprec,
#endif
                   kfang, M_PI/2.0-3*kfrad, 220*kfang/360.0, // angle per node, init angle, total nodes
                   0.01, 1.0, 0.1, 0.1, 5.0); // node noise (m,deg), scale noise (increment),

  cout << "[SPA Spiral] Initial cost is " << spa.calcCost() << endl;
  cout << "[SPA Spiral] Number of constraints is " << spa.p2cons.size() << endl;  

#if 0
  // write out pose results and originals
  cout << "[SPAsys] Writing pose file" << endl;  
  ofstream ofs3("P400.init.txt");
  for (int i=0; i<(int)cps.size(); i++)
    {
      Vector3d tpn = spa.nodes[i].trans.head(3);
      ofs3 << tpn.transpose() << endl;
    }  
  ofs3.close();
#endif


  // add in distance constraint
  // works with either n0 -> ni or ni -> ni+1 constraints
#if 1
  ConScale con;
  con.w = con_weight;                // weight
  for (int i=0; i<(int)cps.size()-3; i++)
    {
      int k = i;
      if (i > 200)
        {
          k = 0;
          con.nd0 = i;              // first node
          con.nd1 = i+1;            // second node
          con.sv  = k;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i+1;            // first node
          con.nd1 = i+2;            // second node
          con.sv  = k;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i+2;            // first node
          con.nd1 = i+3;            // second node
          con.sv  = k;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i;              // first node
          con.nd1 = i+3;            // second node
          con.sv  = k;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);
        }

      else
        {
          spa.scales.push_back(1.0);
          Node nd0;
          Node nd1;

          con.nd0 = i;              // first node
          con.nd1 = i+1;            // second node
          con.sv  = k;              // scale index
          nd0 = spa.nodes[con.nd0];
          nd1 = spa.nodes[con.nd1];
          con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i+1;            // first node
          con.nd1 = i+2;            // second node
          con.sv  = k;              // scale index
          nd0 = spa.nodes[con.nd0];
          nd1 = spa.nodes[con.nd1];
          con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i+2;            // first node
          con.nd1 = i+3;            // second node
          con.sv  = k;              // scale index
          nd0 = spa.nodes[con.nd0];
          nd1 = spa.nodes[con.nd1];
          con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);

          con.nd0 = i;              // first node
          con.nd1 = i+3;            // second node
          con.sv  = k;              // scale index
          nd0 = spa.nodes[con.nd0];
          nd1 = spa.nodes[con.nd1];
          con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);
        }



#if 0                           // doesn't seem to help...
      if (i>2)
        {
          con.nd0 = i-3;              // first node
          con.nd1 = i+1;            // second node
          con.sv  = 0;              // scale index
          con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
          spa.scons.push_back(con);
        }
#endif

    }
#endif


  // add in cross-link distance constraint
  // not much effect here...
#if 0
  {
    ConScale con;
    con.w = 0.1;                // weight
    for (int i=72; i<(int)cps.size()-20; i++)
      {
        con.nd0 = i;              // first node
        con.nd1 = i-2;            // second node
        con.sv  = i-72;           // scale index
        Node nd0 = spa.nodes[con.nd0];
        Node nd1 = spa.nodes[con.nd1];
        con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
        con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
        spa.scons.push_back(con);

        con.nd0 = i-72;
        con.nd1 = i;
        con.sv  = i-72;
        nd0 = spa.nodes[con.nd0];
        nd1 = spa.nodes[con.nd1];
        con.ks  = (nd1.trans.head(3) - nd0.trans.head(3)).squaredNorm(); // measured distance
        con.ks  = (cps[con.nd1].head(3) - cps[con.nd0].head(3)).squaredNorm(); // measured distance
        spa.scons.push_back(con);
      }
  }
#endif



  // this adds in a global constraint connecting the two sides
  // and connecting first and last points

  if (lscale > 0.0)
    {

      // cross-links
#if 0
      {
        ConP2 con;
        con.ndr = 0;
        con.nd1 = 36;
        Node nd0 = spa.nodes[con.ndr];
        Node nd1 = spa.nodes[con.nd1];
        Vector4d trans;
        trans.head(3) = cps[con.nd1].head(3);
        trans(3) = 1.0;

        con.prec = 1000*diagprec;

        con.tmean = lscale * nd0.w2n * trans; // translation offset
        Quaternion<double> q0,q1;
        q0.coeffs() = nd0.qrot;
        q1.coeffs() = nd1.qrot;
        con.qpmean = (q0.inverse()*q1).inverse();

        spa.p2cons.push_back(con);
      }

      {
        ConP2 con;
        con.ndr = 72;
        con.nd1 = 36+72;
        Node nd0 = spa.nodes[con.ndr];
        Node nd1 = spa.nodes[con.nd1];
        Vector4d trans;
        trans.head(3) = cps[con.nd1].head(3);
        trans(3) = 1.0;

        con.prec = 1000*diagprec;

        Quaternion<double> q0,q1;
        q0.vec() = cps[con.ndr].segment(3,3);
        q0.w() = sqrt(1.0 - q0.vec().squaredNorm());
        q1.vec() = cps[con.nd1].segment(3,3);
        q1.w() = sqrt(1.0 - q1.vec().squaredNorm());

        Matrix<double,3,4> w2n;
        Vector4d t0;
        t0.head(3) = cps[con.ndr].head(3);
        t0(3) = 1.0;
        transformW2F(w2n,t0,q0);
        con.tmean = lscale * w2n * trans; // translation offset
        con.qpmean = (q0.inverse()*q1).inverse();

        spa.p2cons.push_back(con);
      }
#endif  // global cross-loop constraint

      // global
      {
        ConP2 con;
        con.ndr = 0;
        con.nd1 = 3*72;
        Node nd0 = spa.nodes[con.ndr];
        Node nd1 = spa.nodes[con.nd1];
        Vector4d trans;
        trans.head(3) = cps[con.nd1].head(3);
        trans(3) = 1.0;

        con.prec = 1000*diagprec;

        con.tmean = lscale * nd0.w2n * trans; // translation offset
        Quaternion<double> q0,q1;

        q0 = nd0.qrot;
        q1 = nd1.qrot;
        con.qpmean = (q0.inverse()*q1).inverse();

        spa.p2cons.push_back(con);
      }
    } // end of global constraints


  {
  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      Vector3d err = tp-tpn;
      sqerr += err.squaredNorm();

      Quaternion<double> qr;
      qr.vec() = cp.block<3,1>(3,0);
      qr.w() = sqrt(1.0 - qr.vec().squaredNorm());
      Quaternion<double> qn;
      qn = spa.nodes[i].qrot;
      double da = qr.angularDistance(qn);
      asqerr += da;

    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));
  }

  // why do we have to draw twice????
  cout << endl << "drawing..." << endl << endl;
  drawgraph(spa,cam_pub,link_pub);
  sleep(1);
  drawgraph(spa,cam_pub,link_pub);
  printf("Press <ret> to continue\n");
  getchar();

  // optimize
  int iters = 20;
  long long t0, t1;
  double ttime = 0.0;
  for (int i=0; i<iters; i++)
    {
      t0 = utime();
      int niters = spa.doSPA(1,0.0);
      t1 = utime();
      ttime += (double)(t1-t0);
      drawgraph(spa,cam_pub,link_pub); 
      sleep(1);
    }

  printf("[TestSPA] Compute took %0.2f ms/iter\n", 0.001*ttime/(double)iters);


#if 0
  // write out A matrix
  spa.setupSys(0.0);
  cout << "[SPAsys] Writing file" << endl;
  spa.writeSparseA("A400mono.txt");
#endif

#if 0
  // write out pose results and originals
  cout << "[SPAsys] Writing pose file" << endl;  
  ofstream ofs1("P400.ground.txt");
  ofstream ofs2("P400.optim.txt");
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      ofs1 << tp.transpose() << endl;
      ofs2 << tpn.transpose() << endl;
    }  
  ofs1.close();
  ofs2.close();
#endif
  

  // test results
  double sqerr = 0.0;
  double asqerr = 0.0;
  for (int i=0; i<(int)cps.size(); i++)
    {
      Matrix<double,6,1> &cp = cps[i]; // old camera pose
      Vector3d tp = cp.head(3);
      Vector3d tpn = spa.nodes[i].trans.head(3);

      //      printf("\n[TestSPA] Cam %d orig: %0.2f %0.2f %0.2f\n", i, tp[0], tp[1], tp[2]);
      //      printf("[TestSPA] Cam %d new:  %0.2f %0.2f %0.2f\n", i, tpn[0], tpn[1], tpn[2]);

      Vector3d err = tp-tpn;
      sqerr += err.squaredNorm();

      Quaternion<double> qr;
      qr.vec() = cp.block<3,1>(3,0);
      qr.w() = sqrt(1.0 - qr.vec().squaredNorm());
      Quaternion<double> qn;
      qn = spa.nodes[i].qrot;
      double da = qr.angularDistance(qn);
      asqerr += da;

    }
  
  sqerr = sqerr / (double)(cps.size());
  asqerr = asqerr / (double)(cps.size());
  printf("\n[TestSPA] RMSE pos is %0.3f m, angle is %0.3f deg\n", 
         sqrt(sqerr), sqrt(asqerr));

  // draw graph
  cout << endl << "drawing..." << endl << endl;
  drawgraph(spa,cam_pub,link_pub); 

  return 0;
}
