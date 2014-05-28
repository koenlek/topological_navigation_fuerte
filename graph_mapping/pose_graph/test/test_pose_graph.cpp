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

#include <pose_graph/constraint_graph.h>
#include <pose_graph/graph_db.h>
#include <graph_mapping_utils/utils.h>
#include <pose_graph/exception.h>
#include <pose_graph/message_conversion.h>
#include <pose_graph/spa_2d_conversion.h>
#include <pose_graph/graph_search.h>
#include <tf/transform_datatypes.h>
#include <gtest/gtest.h>
#include <iostream>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

using namespace pose_graph;
using namespace std;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using boost::assign::operator+=;
using graph_mapping_utils::makePose;

namespace gm=geometry_msgs;

namespace geometry_msgs
{


bool operator==(const gm::Pose& p1, const gm::Pose& p2)
{
  return (p1.position.x == p2.position.x) &&
    (p1.position.y == p2.position.y) &&
    (p1.position.z == p2.position.z) &&
    (p1.orientation.x == p2.orientation.x) &&
    (p1.orientation.y == p2.orientation.y) &&
    (p1.orientation.z == p2.orientation.z) &&
    (p1.orientation.w == p2.orientation.w);
}

} // namespace geometry_msgs


template <class T>
ostream& operator<< (ostream& str, const set<T>& s)
{
  std::copy(s.begin(), s.end(), std::ostream_iterator<unsigned>(str, " "));
  return str;
}

ostream& operator<< (ostream& str, const tf::Pose& p)
{
  const tf::Vector3& v = p.getOrigin();
  const tf::Quaternion& q = p.getRotation();
  str << "[(" << v.x() << ", " << v.y() << ", " << v.z() << "), (" <<
    q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")]";
  return str;
}

template <class S>
bool equalSets (set<S> s1, set<S> s2)
{
  if (s1.size()==s2.size()) {
    BOOST_FOREACH (S x, s1) {
      if (s2.find(x)==s2.end()) {
        return false;
      }
    }
    return true;
  }
  else {
    cout << "\nSets aren't equal:\n  S1: ";
    copy(s1.begin(), s1.end(), ostream_iterator<S>(cout, " "));
    cout << "\n  S2: ";
    copy(s2.begin(), s2.end(), ostream_iterator<S>(cout, " "));
    return false;
  }
}

template <class T>
ostream& operator<< (ostream& str, const vector<T>& s)
{
  std::copy(s.begin(), s.end(), std::ostream_iterator<T>(str, " "));
  return str;
}


void printGraph (const ConstraintGraph& g)
{
  BOOST_FOREACH (const unsigned n, g.allNodes()) {
    cout << "\nNode " << n << ".  Edges: ";
    EdgeSet edges = g.incidentEdges(n);
    copy(edges.begin(), edges.end(), ostream_iterator<unsigned>(cout, " "));
  }
  cout << "\n";
}

const double TOL=1e-3;

bool closeTo (const double x, const double y)
{
  return fabs(x-y) < TOL;
}

bool approxEqualPoses (const tf::Pose& p1, const tf::Pose& p2)
{
  const tf::Vector3& v1 = p1.getOrigin();
  const tf::Vector3& v2 = p2.getOrigin();
  const tf::Quaternion& q1 = p1.getRotation();
  const tf::Quaternion& q2 = p2.getRotation();
  
  return (closeTo(v1.x(), v2.x()) &&
          closeTo(v1.y(), v2.y()) &&
          closeTo(v1.z(), v2.z()) &&
          closeTo(q1.x(), q2.x()) &&
          closeTo(q1.y(), q2.y()) &&
          closeTo(q1.z(), q2.z()) &&
          closeTo(q1.w(), q2.w()));
}


bool approxEqual (const gm::Pose& p1, const gm::Pose& p2)
{
  return approxEqualPoses(graph_mapping_utils::toPose(p1), graph_mapping_utils::toPose(p2));
}

double euclideanDistance (const gm::Point& p1, const gm::Point& p2)
{
  const double dx = p1.x-p2.x;
  const double dy = p1.y-p2.y;
  const double dz = p1.z-p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}


typedef gm::Point::ConstPtr PointPtr;

PointPtr makePoint (const double x, const double y, const double z = 0.0)
{
  gm::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return PointPtr(new gm::Point(p));
}

bool approxEqualPts (const gm::Point& p1, const gm::Point& p2)
{
  return (euclideanDistance(p1, p2) < TOL);
}

unsigned ind(const unsigned i, const unsigned j)
{
  return i*6+j;
}

PoseWithPrecision makeConstraint(double x, double y, double yaw, double x_prec=1,
                                 double y_prec=1, double xy_prec=0, double theta_prec=1)
{
  PoseWithPrecision p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  p.precision[ind(0,0)] = x_prec;
  p.precision[ind(1,1)] = y_prec;
  p.precision[ind(5,5)] = theta_prec/4;
  p.precision[ind(2,2)] = p.precision[ind(3,3)] = p.precision[ind(4,4)] = 1;
  p.precision[ind(0,1)] = p.precision[ind(1,0)] = xy_prec;
  return p;
}


mr::Metadata nodeMetadata (unsigned n)
{
  return mr::Metadata().append("node", n);
}

TEST(PoseGraph, GraphDB)
{
  const unsigned n1(1);
  const unsigned n2(2);
  const unsigned n3(3);
  const unsigned n4(4);

  PointPtr p1 = makePoint(24, 3, 7);
  PointPtr p2 = makePoint(42, 5, 4);
  gm::Pose::Ptr p4(new gm::Pose());
  gm::Pose::Ptr p6(new gm::Pose());
  p4->position = *p2;
  p4->orientation.w = 1.0;
  p6->position = *p2;
  p6->orientation.z = 1.0;
  
  {
    mr::MessageCollection<gm::Point> points("test_pose_graph", "foo");
    points.ensureIndex("x");
    points.ensureIndex("z");

    points.insert(*p1, nodeMetadata(n1));
    EXPECT_PRED2(approxEqualPts, *p1, *getNodeData(&points, n1));
    EXPECT_THROW(getNodeData(&points, n2), DataNotFoundException);

    mr::MessageCollection<gm::Pose> poses("test_pose_graph", "bar");
    poses.insert(*p4, nodeMetadata(n1));
    gm::Pose::ConstPtr p5 = getNodeData(&poses, n1);
    EXPECT_PRED2(approxEqual, *p5, *p4);
  }

  {
    mr::MessageCollection<gm::Point> points("test_pose_graph", "foo");
    points.ensureIndex("x");
    points.ensureIndex("z");
    
    mr::MessageCollection<gm::Pose> poses("test_pose_graph", "bar");
    PointPtr p3 = getNodeData(&points, n1);
    EXPECT_PRED2(approxEqualPts, *p3, *p1);
    EXPECT_THROW(getNodeData(&points, n2), DataNotFoundException);
    gm::Pose::ConstPtr p5 = getNodeData(&poses, n1);
    EXPECT_PRED2(approxEqual, *p5, *p4);
    // EXPECT_THROW(db.getData<gm::Point>("bar", n1), DBTypeMismatchException);


    CachedNodeMap<gm::Point> cached_pts("test_pose_graph", "foo");
    EXPECT_PRED2(approxEqualPts, *p1, *cached_pts.get(n1));
    EXPECT_PRED2(approxEqualPts, *p1, *cached_pts.get(n1));
    EXPECT_THROW(cached_pts.get(n2), DataNotFoundException);
    points.insert(*p2, nodeMetadata(n2));
    ros::Duration(1.0).sleep();
    EXPECT_PRED2(approxEqualPts, *p1, *cached_pts.get(n1));
    EXPECT_PRED2(approxEqualPts, *p2, *cached_pts.get(n2));
    EXPECT_THROW(cached_pts.get(n3), DataNotFoundException);

    // This no longer throws an exception
    // EXPECT_THROW(cached_pts.set(n2, p3), NodeDuplicateDataException);
    
    cached_pts.set(n3, p3);
    EXPECT_PRED2(approxEqualPts, *p3, *cached_pts.get(n3));
    EXPECT_TRUE(cached_pts.hasData(n2));
    EXPECT_TRUE(!cached_pts.hasData(n4));
  }
}

TEST(PoseGraph, GraphOps)
{
  ConstraintGraph g;

  PoseWithPrecision constraint = makeConstraint(1, 2, 0);
  PoseWithPrecision constraint2 = makeConstraint(2, 1, 1);

  const set<unsigned> empty;
  EXPECT_PRED2(equalSets<unsigned>, empty, g.allNodes());


  /****************************************
   * Add a few nodes and edges
   ****************************************/


  unsigned n1 = g.addNode();
  unsigned n2 = g.addNode();
  unsigned n3 = g.addNode();
  unsigned e12 = g.addEdge(n1, n2, constraint);
  unsigned e23 = g.addEdge(n2, n3, constraint2);
  unsigned e23b = g.addEdge(n2, n3, constraint);

  EdgeSet edges1, edges2, edges;
  edges1.insert(e12);
  edges2.insert(e12);
  edges2.insert(e23);
  edges2.insert(e23b);
  edges.insert(e12);
  edges.insert(e23);
  edges.insert(e23b);

  NodeSet nodes;
  nodes.insert(n1);
  nodes.insert(n2);
  nodes.insert(n3);

  /****************************************
   * Check they were added correctly
   ****************************************/

  EXPECT_TRUE(e23!=e23b);
  EXPECT_TRUE(equalSets(g.incidentEdges(n1), edges1));
  EXPECT_TRUE(equalSets(g.incidentEdges(n2), edges2));

  /****************************************
   * Optimized pose getting and setting
   ****************************************/
  
  const tf::Pose p1(tf::Quaternion(0,0,0,1), tf::Vector3(0, 42.24, 0));
  const tf::Pose p2(tf::Quaternion(0,0,1,0), tf::Vector3(0, 100, 200));
  EXPECT_THROW (g.getOptimizedPose(n1), NoOptimizedPoseException);
  g.setOptimizedPose(n1, p1);
  const tf::Pose p3 = g.getOptimizedPose(n1);
  EXPECT_PRED2 (approxEqualPoses, p1, p3);
  EXPECT_THROW (g.getOptimizedPose(n2), NoOptimizedPoseException);
  g.setOptimizedPose(n2, p2);
  EXPECT_PRED2 (approxEqualPoses, p2, g.getOptimizedPose(n2));
  EXPECT_PRED2 (approxEqualPoses, p1, g.getOptimizedPose(n1));
  NodePoseMap poses;
  poses[n2] = p2;
  g.setOptimizedPoses(poses);
  EXPECT_PRED2 (approxEqualPoses, p2, g.getOptimizedPose(n2));
  EXPECT_THROW (g.getOptimizedPose(n1), NoOptimizedPoseException);
  
  /****************************************
   * Test the copy constructor
   ****************************************/

  const ConstraintGraph g2(g);

  EXPECT_TRUE(equalSets(g2.incidentEdges(n1), edges1));
  EXPECT_TRUE(equalSets(g2.incidentEdges(n2), edges2));
  EXPECT_TRUE(equalSets(g2.allEdges(), edges));
  EXPECT_TRUE(equalSets(g2.allNodes(), nodes));

  /****************************************
   * Conversion to and from ros messages
   ****************************************/

  
  msg::ConstraintGraphMessage m = constraintGraphToMessage(g2);
  ConstraintGraph g3 = constraintGraphFromMessage(m);
  
  EXPECT_TRUE(equalSets(g3.incidentEdges(n1), edges1));
  EXPECT_TRUE(equalSets(g3.incidentEdges(n2), edges2));
  EXPECT_TRUE(equalSets(g3.allEdges(), edges));
  EXPECT_TRUE(equalSets(g3.allNodes(), nodes));

  ConstraintGraph g4(g3);

  /****************************************
   * Verify optimized poses still there
   ****************************************/
  
  EXPECT_PRED2 (approxEqualPoses, p2, g4.getOptimizedPose(n2));
  EXPECT_THROW (g4.getOptimizedPose(n1), NoOptimizedPoseException);


  /****************************************
   * Test nearbyNodes
   ****************************************/

  const tf::Pose pose3(tf::Quaternion(), tf::Vector3(0, 500, 500));
  const tf::Pose pose4(tf::Quaternion(), tf::Vector3(0, 100, 150));
  const unsigned n4 = g4.addNode();
  g4.addNode();
  const unsigned n6 = g4.addNode();
  g4.addEdge(n3, n4, constraint2);
  g4.setOptimizedPose(n3, pose3);
  g4.setOptimizedPose(n4, pose4);
  g4.setOptimizedPose(n1, p1);
  g4.setOptimizedPose(n6, p1);
  NodeSet neighbors2;
  neighbors2.insert(n2);
  neighbors2.insert(n1);
  OptimizedDistancePredicate pred(g4, p2.getOrigin(), 300);
  EXPECT_TRUE(equalSets(filterNearbyNodes(g4, n2, pred), neighbors2));

  /****************************************
   * Shortest paths
   ****************************************/
  
  const unsigned e36 = g4.addEdge(n3, n6, constraint2);
  g4.addEdge(n4, n6, constraint);
  ShortestPath path = shortestPath(g4, n1, n6);
  ShortestPath exp_path;
  exp_path.first += n1, n2, n3, n6;
  exp_path.second += e12, e23, e36;
  EXPECT_EQ (exp_path.first, path.first);
  EXPECT_EQ (exp_path.second, path.second);

  /****************************************
   * Diffs
   ****************************************/
  
  msg::ConstraintGraphDiff diff;
  diff.new_nodes.resize(1);
  diff.new_node_timestamps.resize(1);
  diff.new_nodes[0].id = 42;
  diff.new_edges.resize(1);
  diff.new_edges[0].id = 100;
  diff.new_edges[0].constraint.src = n1;
  diff.new_edges[0].constraint.dest = n3;
  EXPECT_EQ(g4.allNodes().size(), 6);
  EXPECT_EQ(g4.incidentEdges(n1).size(), 1);
  applyDiff(&g4, diff);
  EXPECT_EQ(g4.allNodes().size(), 7);
  EXPECT_EQ(g4.incidentEdges(n1).size(), 2);

}



double dist (const tf::Pose& p, const tf::Pose& p2)
{
  const double dx=p.getOrigin().x()-p2.getOrigin().x();
  const double dy=p.getOrigin().y()-p2.getOrigin().y();
  return sqrt(dx*dx+dy*dy);
}



TEST(PoseGraph, Optimization) 
{
  ConstraintGraph g;

  const unsigned n1 = g.addNode();
  const unsigned n2 = g.addNode();
  const unsigned n3 = g.addNode();

  Eigen::Matrix<double, 6, 6> p1, p2;
  p1 = Eigen::MatrixXd::Identity(6,6);
  p2 = p1;

  const PoseWithPrecision c12 = makeConstraint(1, 0, 0);
  const PoseWithPrecision c13 = makeConstraint(0.5, 1, 0);
  const PoseWithPrecision c32 = makeConstraint(0.5, -1, 0);

  g.addEdge(n1, n2, c12);
  g.addEdge(n1, n3, c13);
  g.addEdge(n3, n2, c32);

  NodePoseMap init;
  init[n1] = makePose(0, 0, 0);
  init[n2] = makePose(2, 5, 0);
  init[n3] = makePose(9, 1, 0);
  /*
  NodePoseMap opt=optimizeGraph(g, init);
  
  EXPECT_TRUE (0.98 < dist(opt[n1], opt[n2]));
  EXPECT_TRUE (dist(opt[n1], opt[n2]) < 1.02);
  EXPECT_TRUE (1.1 < dist(opt[n3], opt[n2])); 
  EXPECT_TRUE (dist(opt[n3], opt[n2]) < 1.2);
  EXPECT_TRUE (1.1 < dist(opt[n1], opt[n3]));
  EXPECT_TRUE (dist(opt[n1], opt[n3]) < 1.2);
  */

  NodePoseMap opt2d=optimizeGraph2D(g, init);
  
  EXPECT_TRUE (0.98 < dist(opt2d[n1], opt2d[n2]));
  EXPECT_TRUE (dist(opt2d[n1], opt2d[n2]) < 1.02);
  EXPECT_TRUE (1.1 < dist(opt2d[n3], opt2d[n2])); 
  EXPECT_TRUE (dist(opt2d[n3], opt2d[n2]) < 1.2);
  EXPECT_TRUE (1.1 < dist(opt2d[n1], opt2d[n3]));
  EXPECT_TRUE (dist(opt2d[n1], opt2d[n3]) < 1.2);

  /// Test that disconnected graphs throw exception
  const unsigned n4 = g.addNode();
  init[n4] = makePose(0, 1, 0);
  EXPECT_THROW(optimizeGraph2D(g, init), DisconnectedComponentException);
  NodeSet nodes1, nodes2, nodes3;
  nodes1 += n1, n2, n3;
  nodes2 += n4;
  nodes3 += n1, n2, n3, n4;
  opt2d = optimizeGraph2D(g, init, nodes1);
  EXPECT_EQ (3, opt2d.size());
  
  EXPECT_PRED2(equalSets<unsigned>, nodes1, componentContaining(g, n2));
  EXPECT_PRED2(equalSets<unsigned>, nodes2, componentContaining(g, n4));

  /// Once it's connected, it should work
  g.addEdge(n2, n4, c12);
  opt2d = optimizeGraph2D(g, init);
  EXPECT_TRUE (0.98 < dist(opt2d[n1], opt2d[n2]));
  EXPECT_TRUE (dist(opt2d[n1], opt2d[n2]) < 1.02);
  EXPECT_TRUE (1.1 < dist(opt2d[n3], opt2d[n2])); 
  EXPECT_TRUE (dist(opt2d[n3], opt2d[n2]) < 1.2);
  EXPECT_TRUE (1.1 < dist(opt2d[n1], opt2d[n3]));
  EXPECT_TRUE (dist(opt2d[n1], opt2d[n3]) < 1.2);
  EXPECT_TRUE (0.99 < dist(opt2d[n2], opt2d[n4]));
  EXPECT_TRUE (dist(opt2d[n2], opt2d[n4]) < 1.01);

  EXPECT_PRED2(equalSets<unsigned>, nodes3, componentContaining(g, n2));  
}


const double PI = 3.14159265;


int main (int argc, char** argv)
{
  ros::init(argc, argv, "pose_graph_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
