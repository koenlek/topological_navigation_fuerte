/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * 
 * Test the scan matcher constraint generation
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/pose_graph.h>
#include <pose_graph/pose_graph_message.h>
#include <graph_slam/constraints/karto_laser_constraints.h>
#include <pose_graph/geometry.h>
#include <pose_graph/transforms.h>
#include <pose_graph/exception.h>
#include <pose_graph/spa_conversion.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
#include <gtest/gtest.h>
#include <iostream>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>

namespace pg=pose_graph;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace ksm=karto_scan_matcher;
namespace gs=graph_slam;

using boost::assign::operator+=;
using std::vector;
using pg::NodeId;
using std::ostream;

const double PI=3.14159265;

typedef std::map<pg::NodeId, sm::LaserScan> NodeScanMap;
typedef std::vector<gs::Chain> ChainVec;

gm::Pose makePose (const double x, const double y, const double theta=0)
{
  return pg::convertToPose(pg::makePose2D(x, y, theta));
}

template <class T>
bool equalVectors (const vector<T>& v1, const vector<T>& v2)
{
  if (v1.size() != v2.size())
    return false;
  for (unsigned i=0; i<v1.size(); i++) {
    if (v1[i] != v2[i])
      return false;
  }
  return true;
}

template <class T>
ostream& operator<< (ostream& str, const vector<T>& s)
{
  std::copy(s.begin(), s.end(), std::ostream_iterator<T>(str, " "));
  return str;
}


TEST(ScanMatcher, LoopClosure)
{
  sm::LaserScan* scan_ptr = new sm::LaserScan();
  sm::LaserScan::ConstPtr scan(scan_ptr);
  scan_ptr->angle_min = -PI/2;
  scan_ptr->angle_max = PI/2;
  scan_ptr->angle_increment = PI;
  scan_ptr->range_max = 10;
  scan_ptr->range_min = 0.0;
  scan_ptr->ranges += 1.0, 1.0;

  vector<pg::NodeId> nodes;
  const pg::PoseConstraint c; // value is irrelevant
  pg::PoseGraph g;
  gm::Pose2D offset;
  gs::LoopScanMatcher matcher(offset, 1.7, 3, 0.8, true);

  // Build up graph
  for (unsigned i=0; i<16; i++) {
    nodes.push_back(g.addNode());
    if (i>0) {
      g.addEdge(nodes[i-1], nodes[i], c);
    }
    matcher.addNode(nodes[i], scan);
  }
  nodes.push_back(g.addNode());
  
  // Set optimized poses
  pg::NodePoseMap opt_poses;
  unsigned ind=0;
  opt_poses[nodes[ind++]] = makePose(0, 2.0, 0);
  opt_poses[nodes[ind++]] = makePose(0.1, 2.0, 0);
  opt_poses[nodes[ind++]] = makePose(0.5, 0.1, 0);
  opt_poses[nodes[ind++]] = makePose(1, 1, 0);
  opt_poses[nodes[ind++]] = makePose(0, 1, 0);
  for (double y=1; y>-1.1; y-=1)
    opt_poses[nodes[ind++]] = makePose(-1, y, 0);
  for (double x=0; x<3.5; x+=1)
    opt_poses[nodes[ind++]] = makePose(x, -1, 0);
  for (double x=3; x>-0.1; x-=1)
    opt_poses[nodes[ind++]] = makePose(x, 0, 0);

  /**

  gs::WithOptimizedPoses opt(&matcher, opt_poses);
  gs::ChainVec chains = matcher.possibleLoopClosures (g, nodes[15], makePose(0.1, 0.1), scan);

  gs::Chain c1;
  c1 += nodes[9], nodes[8], nodes[7], nodes[6], nodes[5], nodes[4], nodes[3], nodes[2];
  EXPECT_EQ(1u, chains.size());
  EXPECT_PRED2(equalVectors<pg::NodeId>, c1, chains[0]);
  */
}


TEST(ScanMatcher, Sequential)
{
  sm::LaserScan* scan_ptr = new sm::LaserScan();
  sm::LaserScan::ConstPtr scan(scan_ptr);
  scan_ptr->angle_min = -PI/2;
  scan_ptr->angle_max = PI/2;
  scan_ptr->angle_increment = PI;
  scan_ptr->range_max = 10;
  scan_ptr->range_min = 0.0;
  scan_ptr->ranges += 1.0, 1.0;

  vector<pg::NodeId> nodes;
  const pg::PoseConstraint c; // value is irrelevant
  pg::PoseGraph g;
  gm::Pose2D offset;
  gs::SequentialScanMatcher matcher(offset, 6.1, 3, 0.1, true);

  for (unsigned i=0; i<21; i++) {
    nodes.push_back(g.addNode());
    if (i>0) {
      g.addEdge(nodes[i-1], nodes[i], c);
    }
    matcher.addNode(nodes[i], scan);
  }
  nodes.push_back(g.addNode());

  // Loop
  g.addEdge(nodes[0], nodes[18], c);

  pg::NodePoseMap opt_poses;
  unsigned ind=0;
  for (double x=11; x>2; x-=2)
    opt_poses[nodes[ind++]] = makePose(x, 0, 0);
  for (double x=1; x>-6; x-=2)
    opt_poses[nodes[ind++]] = makePose(x, 1, 0);
  for (double x=-5; x<12; x+=2)
    opt_poses[nodes[ind++]] = makePose(x, 2, 0);
  for (double x=11; x>6; x-=2)
    opt_poses[nodes[ind++]] = makePose(x, 1, 0);
  /*
   Commenting out pending change to candidateChains to use barycenter
   Need to get this test case working again
  gs::WithOptimizedPoses opt(&matcher, opt_poses);
  gs::ChainVec v1 = matcher.candidateChains(g, nodes[20], makePose(5, 1, 0), scan);

  gs::Chain c1, c2;
  c1 += nodes[0], nodes[1], nodes[2], nodes[3], nodes[4], nodes[5], nodes[6];
  c2 += nodes[11], nodes[12], nodes[13], nodes[14], nodes[15],
    nodes[16], nodes[17], nodes[18], nodes[19], nodes[20];

  // Test that constraints look right
  EXPECT_EQ(2u, v1.size());
  EXPECT_PRED2(equalVectors<pg::NodeId>, c1, v1[0]);
  EXPECT_PRED2(equalVectors<pg::NodeId>, c2, v1[1]);

  */

  // Not necessary
  g.setInitialPoseEstimate(nodes[21], makePose(5, 1, 0));
  matcher.addNode(nodes[21], scan);
}


        
     

int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
