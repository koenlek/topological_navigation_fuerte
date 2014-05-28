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

/**
 * \file 
 * 
 * Implementation for constraint_graph.h
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/exception.h>
#include <graph_mapping_utils/utils.h>
#include <graph_mapping_msgs/msg_utils.h>
#include <boost/foreach.hpp>
#include <pose_graph/constraint_graph.h>

namespace pose_graph
{

namespace util=graph_mapping_utils;
namespace msg=graph_mapping_msgs;

using std::pair;
using std::map;

typedef pair<unsigned, unsigned> IncidentNodes;
typedef map<unsigned, GraphVertex> NodeMap;
typedef map<unsigned, GraphEdge> EdgeMap;

/************************************************************
 * Construction
 ***********************************************************/

ConstraintGraph::ConstraintGraph () :
  next_node_id_(1), next_edge_id_(1)
{}

ConstraintGraph::~ConstraintGraph () 
{}

ConstraintGraph::ConstraintGraph (const ConstraintGraph& other)
{
  ROS_DEBUG_STREAM_NAMED ("copy", "Copy constructing constraint graph");
  initializeFrom(other);
}

ConstraintGraph& ConstraintGraph::operator= (const ConstraintGraph& other)
{
  ROS_DEBUG_STREAM_NAMED ("copy", "Assigning constraint graph");
  if (this != &other)
    initializeFrom(other);
  return *this;
}

void ConstraintGraph::initializeFrom (const ConstraintGraph& g)
{
  /// Clear out data associated with old graph
  graph_.clear();
  next_node_id_ = 1;
  next_edge_id_ = 1;
  vertex_map_.clear();
  edge_map_.clear();

  NodeSet nodes = g.allNodes();

  BOOST_FOREACH (const unsigned n, nodes) {
    addNode(n);
    if (g.hasOptimizedPose(n)) {
      setOptimizedPose(n, g.getOptimizedPose(n));
    }
  }

  BOOST_FOREACH (const unsigned n, nodes) {
    BOOST_FOREACH (const unsigned e, g.incidentEdges(n)) {
      IncidentNodes incident_nodes = g.incidentNodes(e);
      if (n == incident_nodes.first)
        addEdge(e, n, incident_nodes.second, g.getConstraint(e));
    }
  }
}


/************************************************************
 * Basic modification
 ***********************************************************/

unsigned ConstraintGraph::addNode ()
{
  while (nodeIdExists(next_node_id_))
    next_node_id_++;
  addNode(next_node_id_);
  return next_node_id_;
}

void ConstraintGraph::addNode (const unsigned id)
{
  if (nodeIdExists(id))
    throw DuplicateNodeIdException(id);
  ROS_ASSERT(id>0);
  vertex_map_[id] = add_vertex(util::makeNode(id), graph_);
  ROS_DEBUG_STREAM_NAMED ("graph_add_node", "Added node " << id);
}

unsigned ConstraintGraph::addEdge (const unsigned from, const unsigned to, const PoseWithPrecision& constraint)
{
  while (edgeIdExists(next_edge_id_))
    next_edge_id_++;
  addEdge(next_edge_id_, from, to, constraint);
  return next_edge_id_;
}


void ConstraintGraph::addEdge (const unsigned id, const unsigned from, const unsigned to,
                               const PoseWithPrecision& constraint)
{
  const GraphVertex from_vertex = idVertex(from);
  const GraphVertex to_vertex = idVertex(to);
  if (edgeIdExists(id))
    throw DuplicateEdgeIdException(id);
  pair<GraphEdge, bool> result =
    add_edge(from_vertex, to_vertex, util::makeEdge(id, from, to, constraint), graph_);
  ROS_ASSERT(result.second);
  edge_map_[id] = result.first;
  ROS_DEBUG_STREAM_NAMED ("graph_add_edge", "Added edge " << id << " from " << from << " to " << to);
}

/************************************************************
 * Basic const ops
 ***********************************************************/

NodeSet ConstraintGraph::allNodes () const
{
  NodeSet nodes;
  BOOST_FOREACH (const NodeMap::value_type& e, vertex_map_) 
    nodes.insert(e.first);
  return nodes;
}

EdgeSet ConstraintGraph::allEdges () const
{
  EdgeSet edges;
  BOOST_FOREACH (const EdgeMap::value_type& e, edge_map_) {
    edges.insert(e.first);
  }
  return edges;
}

EdgeSet ConstraintGraph::incidentEdges (const unsigned n) const
{
  EdgeSet edges;
  GraphVertex v = idVertex(n);
  BOOST_FOREACH (const GraphEdge& e, out_edges(v, graph_))
    edges.insert(graph_[e].id);
  return edges;
}

IncidentNodes ConstraintGraph::incidentNodes (const unsigned e) const
{
  GraphEdge edge = idEdge(e);
  const unsigned n1 = graph_[source(edge, graph_)].id;
  const unsigned n2 = graph_[target(edge, graph_)].id;
  const msg::Edge& info = graph_[edge];

  if (info.constraint.src == n1) {
    ROS_ASSERT(info.constraint.dest == n2);
    return IncidentNodes(n1, n2);
  }
  else {
    ROS_ASSERT(info.constraint.dest == n1 && info.constraint.src == n2);
    return IncidentNodes(n2, n1);
  }
}

const PoseWithPrecision& ConstraintGraph::getConstraint (const unsigned e) const
{
  return graph_[idEdge(e)].constraint.constraint;
}

/************************************************************
 * Ops on underlying graph
 ***********************************************************/

const Graph& ConstraintGraph::graph () const
{
  return graph_;
}

GraphEdge ConstraintGraph::idEdge (const unsigned e) const
{
  EdgeMap::const_iterator pos = edge_map_.find(e);
  if (pos == edge_map_.end())
    throw UnknownEdgeIdException(e);
  return pos->second;
}

GraphVertex ConstraintGraph::idVertex (const unsigned e) const
{
  NodeMap::const_iterator pos = vertex_map_.find(e);
  if (pos == vertex_map_.end())
    throw UnknownNodeIdException(e);
  return pos->second;
}

/************************************************************
 * Optimized poses
 ***********************************************************/

tf::Pose ConstraintGraph::getOptimizedPose (unsigned n) const
{
  const msg::Node& node_info = graph_[idVertex(n)];
  if (node_info.optimized_pose.size()==1) {
    return util::toPose(node_info.optimized_pose[0]);
  }
  ROS_ASSERT (node_info.optimized_pose.size()==0);
  throw NoOptimizedPoseException(n);
}

void ConstraintGraph::setOptimizedPose (const unsigned n, const tf::Pose& pose) 
{
  msg::Node& info = graph_[idVertex(n)];
  info.optimized_pose.resize(1);
  info.optimized_pose[0] = util::toPose(pose);
}

void ConstraintGraph::setOptimizedPoses (const NodePoseMap& poses)
{
  typedef map<unsigned, GraphVertex> VertexMap;
  BOOST_FOREACH (const VertexMap::value_type& e, vertex_map_) {
    NodePoseMap::const_iterator pos = poses.find(e.first);
    if (pos==poses.end())
      graph_[e.second].optimized_pose.resize(0);
    else {
      graph_[e.second].optimized_pose.resize(1);
      graph_[e.second].optimized_pose[0] = util::toPose(pos->second);
    }
  }  
}

bool ConstraintGraph::hasOptimizedPose (unsigned n) const
{
  return (graph_[idVertex(n)].optimized_pose.size()==1);
}

/************************************************************
 * Utilities
 ***********************************************************/

bool ConstraintGraph::nodeIdExists (const unsigned n) const
{
  return util::contains(vertex_map_, n);
}

bool ConstraintGraph::edgeIdExists (const unsigned e) const
{
  return util::contains(edge_map_, e);
}


} // namespace


