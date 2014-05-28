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
 * Implementation for message_conversion.h
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/message_conversion.h>
#include <graph_mapping_utils/geometry.h>
#include <boost/foreach.hpp>

namespace pose_graph
{

msg::ConstraintGraphMessage constraintGraphToMessage (const ConstraintGraph& g)
{
  msg::ConstraintGraphMessage m;
  NodeSet nodes = g.allNodes();
  ROS_DEBUG_STREAM_NAMED ("message_conversion", "Converting graph with " << nodes.size() <<
                          " nodes to message");
  m.nodes.clear();
  m.edges.clear();
  BOOST_FOREACH (const unsigned n, nodes) {
    msg::Node node;
    node.id = n;
    if (g.hasOptimizedPose(n)) {
      node.optimized_pose.resize(1);
      node.optimized_pose[0] = graph_mapping_utils::toPose(g.getOptimizedPose(n));
    }
    m.nodes.push_back(node);
  }

  BOOST_FOREACH (const unsigned e, g.allEdges()) {
    msg::Edge edge;
    edge.id = e;
    std::pair<unsigned, unsigned> nodes = g.incidentNodes(e);
    edge.constraint.src = nodes.first;
    edge.constraint.dest = nodes.second;
    edge.constraint.constraint = g.getConstraint(e);
    m.edges.push_back(edge);
  }
  return m;
}

ConstraintGraph constraintGraphFromMessage (const msg::ConstraintGraphMessage& m)
{
  ConstraintGraph g;
  ROS_DEBUG_STREAM_NAMED ("message_conversion", "Converting graph from message with " << m.nodes.size() <<
                          " nodes ");
  ROS_ASSERT (g.allNodes().empty());
  BOOST_FOREACH (const msg::Node& n, m.nodes) {
    const unsigned id = n.id;
    g.addNode(id);
    if (n.optimized_pose.size() == 1)
      g.setOptimizedPose(id, graph_mapping_utils::toPose(n.optimized_pose[0]));
  }

  BOOST_FOREACH (const msg::Edge& e, m.edges)
    g.addEdge(e.id, e.constraint.src, e.constraint.dest, e.constraint.constraint);
  
  return g;
}

void applyDiff (ConstraintGraph* g, const msg::ConstraintGraphDiff& diff)
{
  ROS_ASSERT (diff.new_nodes.size() == diff.new_node_timestamps.size());
  BOOST_FOREACH (const msg::Node& n, diff.new_nodes) {
    g->addNode(n.id);
  }
  BOOST_FOREACH (const msg::Edge& e, diff.new_edges) {
    g->addEdge(e.id, e.constraint.src, e.constraint.dest, e.constraint.constraint);
  }
}


} // namespace pose_graph


