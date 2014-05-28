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
 *
 */


#include <pose_graph/spa_2d_conversion.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/general.h>
#include <pose_graph/exception.h>
#include <pose_graph/graph_search.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/connected_components.hpp>
#include <set>

namespace pose_graph
{

namespace util=graph_mapping_utils;

using std::set;
using boost::tie;
using sba::Node2d;
using sba::SysSPA2d;
using sba::Con2dP2;

typedef std::pair<unsigned, unsigned> NodePair;


Eigen::Vector4d quaternionMsgToVector4d (const geometry_msgs::Quaternion& m)
{
  return Eigen::Vector4d(m.x, m.y, m.z, m.w);
}

Eigen::Vector4d translationToVector4d(const geometry_msgs::Point& p)
{
  return Eigen::Vector4d(p.x, p.y, p.z, 1.0);
}


// Make a spa node given an initial pose estimate
Node2d makeNode (const double x, const double y, const double theta)
{
  Node2d n;
  n.trans(0) = x;
  n.trans(1) = y;
  n.trans(2) = 1.0;
  n.arot = theta;
  n.setTransform();
  n.setDr();
  return n;
}


// Get yaw from an eigen quaternion
double getYaw (const Quaterniond& q)
{
  const tf::Quaternion qt(q.x(), q.y(), q.z(), q.w());
  return tf::getYaw(qt);
}

unsigned index(const unsigned i, const unsigned j)
{
  return 6*i + j;
}

Con2dP2 makeSpa2DConstraint (const PoseWithPrecision& constraint)
{
  Con2dP2 spa;
  spa.tmean[0] = constraint.pose.position.x;
  spa.tmean[1] = constraint.pose.position.y;
  spa.amean = tf::getYaw(constraint.pose.orientation);

  spa.prec = Matrix3d::Zero();
  spa.prec(0,0) = constraint.precision[index(0,0)];
  spa.prec(1,1) = constraint.precision[index(1,1)];

  // The 6x6 matrix constraint.precision is in order x,y,z of position and x,y,z of quaternion orientation. 
  // The line below would be correct for diagonal precision matrices, if theta = z/2, but in fact theta=sin(z/2)
  // so this isn't quite right for theta far from 0.  The precisions are heuristically set anyway though...
  spa.prec(2,2) = constraint.precision[index(5,5)]*4;

  return spa;
}


void verifyConnected (const ConstraintGraph& g, const NodeSet& nodes)
{
  if (nodes.size() > 0) {
    const unsigned n = *nodes.begin();
    const NodeSet comp = componentContaining(g, n);
    BOOST_FOREACH (const unsigned n2, nodes) {
      if (!util::contains(comp, n2))
        throw DisconnectedComponentException(n, n2);
    }
  }
}


// Turn a pose graph into a pointer to a Spa2d
Spa2DConversionResult constraintGraphToSpa2D (const ConstraintGraph& g, const NodePoseMap& init,
                                              const NodeSet& nodes)
{
  Spa2DPtr spa(new SysSPA2d());
  map<unsigned, unsigned> node_indices;
  unsigned next_index=0;

  // Check if component is connected and throw an exception if not
  verifyConnected(g, nodes);

  // 1. Add the nodes
  BOOST_FOREACH (const unsigned n, nodes) {
    const tf::Pose p = util::keyValue(init, n);
    spa->nodes.push_back(makeNode(p.getOrigin().x(), p.getOrigin().y(), tf::getYaw(p.getRotation())));
    node_indices[n] = next_index++;
  }

  // 2. Add the edge constraints
  // \todo Probably faster to iterate over just the outgoing edges of nodes in this component
  BOOST_FOREACH (const unsigned e, g.allEdges()) {

    NodePair ids = g.incidentNodes(e);
    if (util::contains(nodes, ids.first) && util::contains(nodes, ids.second)) {
    const PoseWithPrecision constraint = g.getConstraint(e);
    Con2dP2 spa_constraint = makeSpa2DConstraint(constraint);

    spa_constraint.ndr = node_indices[ids.first];
    spa_constraint.nd1 = node_indices[ids.second];
    spa->p2cons.push_back(spa_constraint);
    ROS_DEBUG_STREAM_NAMED ("pose_graph_spa_int", "Adding constraint tmean "
                            << spa_constraint.tmean << " and amean "
                            << spa_constraint.amean << " from "
                            << spa_constraint.ndr << " to "
                            << spa_constraint.nd1);
    }
  }

  
  return Spa2DConversionResult(spa, node_indices);
}


// Return the Pose corresponding to a spa node
tf::Pose getNodePose (const sba::Node2d& n)
{
  return util::makePose(n.trans(0), n.trans(1), n.arot);
}


// Convert and optimize a graph
NodePoseMap optimizeGraph2D (const ConstraintGraph& g, const NodePoseMap& init)
{
  return optimizeGraph2D(g, init, g.allNodes());
}

/// Optimize subset of nodes
NodePoseMap optimizeGraph2D (const ConstraintGraph& g, const NodePoseMap& init,
                             const NodeSet& nodes)
{
  /******************************
   * 1. Convert graph to SPA2D
   ******************************/

  ROS_DEBUG_NAMED ("pose_graph_spa", "Converting component with %zu nodes to spa", nodes.size());
  Spa2DConversionResult res=constraintGraphToSpa2D(g, init, nodes);

  /******************************
   * 2. Optimize
   ******************************/

  const size_t num_nodes = res.spa->nodes.size();
  const size_t num_constraints = res.spa->p2cons.size();

  // Shouldn't need this check, but spa dies otherwise
  if (num_nodes>=2 && num_constraints>=1) { 
    ROS_DEBUG_NAMED ("pose_graph_spa", "Performing spa with %zu nodes and %zu constraints",
                     num_nodes, num_constraints);
    res.spa->doSPA(10, 1.0e-4, SBA_DENSE_CHOLESKY);
    ROS_DEBUG_NAMED ("pose_graph_spa", "Graph optimization complete");
  }
  else 
    ROS_DEBUG_NAMED ("pose_graph_spa", "Not optimizing graph with %zu nodes and %zu constraints",
                     num_nodes, num_constraints);
  
  /******************************
   * 3. Return optimized poses
   ******************************/

  NodePoseMap optimized_poses;
  BOOST_FOREACH (const NodeIndexMap::value_type entry, res.node_index_map) {
    optimized_poses[entry.first] = getNodePose(res.spa->nodes[entry.second]);
  }
  ROS_ASSERT(optimized_poses.size() == nodes.size());
  return optimized_poses;
}




} // namespace
