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
 * Graph algorithms
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_GRAPH_SEARCH_H
#define POSE_GRAPH_GRAPH_SEARCH_H

#include <pose_graph/constraint_graph.h>
#include <graph_mapping_utils/ros.h>

namespace pose_graph
{

/// \return The largest connected component of the graph that 1) includes \a n 2) contains only nodes that
/// satisfy the predicate, 
/// \tparam Predicate a default-constructible type with operator() (GraphVertex) const defined
template <class Predicate>
NodeSet filterNearbyNodes (const ConstraintGraph& g, const unsigned n, const Predicate& p);

/// \return The largest connected component of the graph that 1) includes \a n 2) contains only nodes that
/// satisfy the predicate 3) uses only edges that satisfy the predicate
/// \tparam NodePredicate a default-constructible type with operator() (GraphVertex) const defined
/// \tparam EdgePredicate a default-constructible type with operator() (GraphEdge) const defined
template <class NodePredicate, class EdgePredicate>
NodeSet filterNearbyNodes (const ConstraintGraph& g, const unsigned n, const NodePredicate& node_pred,
                           const EdgePredicate& edge_pred);



/// Predicate used with filterNearbyNodes to find nodes whose optimized
/// position is near some point
struct OptimizedDistancePredicate
{
  /// Initialization arguments are stored by reference and must be valid
  /// whenever this predicate is used
  OptimizedDistancePredicate (const ConstraintGraph& g,
                              const tf::Vector3& p, const double window_size) :
    g(&g.graph()), p(&p), squared_window_size(window_size * window_size) 
  {}

  OptimizedDistancePredicate () : g(NULL), p(NULL) {}

  bool operator() (const GraphVertex v) const
  {
    if ((*g)[v].optimized_pose.size()==1) {
      const geometry_msgs::Point& pt = (*g)[v].optimized_pose[0].position;
      const double dx = p->x() - pt.x;
      const double dy = p->y() - pt.y;
      const double dz = p->z() - pt.z;
      ROS_DEBUG_NAMED ("optimized_distance", "Checking optimized distance "
                       "between node %u at (%.2f, %.2f) and (%.2f, %.2f)",
                       (unsigned) (*g)[v].id, pt.x, pt.y, p->x(), p->y());
      return (dx*dx + dy*dy + dz*dz < squared_window_size);
    }
    else {
      ROS_DEBUG_NAMED ("optimized_distance",
                       "Node %u did not have an optimized pose", (*g)[v].id);
      return false;
    }
  }

  const Graph* g;
  const tf::Vector3* p;
  double squared_window_size;
};


/// dummy predicate that always returns true for edges
struct AllEdges 
{
  bool operator() (const GraphEdge&) const { return true; }
};


/// \retval A vector of sets of nodes, each of which is one of the connected
/// components of the graph
std::vector<NodeSet>
connectedComponents (const ConstraintGraph& g);

/// \retval Component containing node \a n
NodeSet componentContaining (const ConstraintGraph& g, unsigned n);


typedef std::pair<std::vector<unsigned>, std::vector<unsigned> > ShortestPath;

/// \retval Shortest path between two nodes, where edge cost is
/// magnitude of relative pose attached to edge constraint.
/// \todo Use covariance information here
/// \throw NoPathException
ShortestPath shortestPath (const ConstraintGraph& g,
                           const unsigned src, const unsigned dest);

/// \retval Relative pose of \a n2 w.r.t \a n1 formed by using
/// constraints along shortest path
/// \throw NoPathException
tf::Pose relativePose (const ConstraintGraph& g,
                       const unsigned n1, const unsigned n2);



} // namespace

#include "../../src/graph_search_impl.h"

#endif // include guard
