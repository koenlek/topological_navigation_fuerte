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
 * Defines the constraint graph type, which represents a set of poses and constraints,
 * but does not contain any other information (e.g., sensor data)
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_CONSTRAINT_GRAPH_H
#define POSE_GRAPH_CONSTRAINT_GRAPH_H

#include <graph_mapping_msgs/Node.h>
#include <graph_mapping_msgs/Edge.h>

#include <tf/transform_datatypes.h>

#define BOOST_NO_HASH
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace pose_graph
{

namespace msg=graph_mapping_msgs;
using msg::PoseWithPrecision;


/************************************************************
 * Definition of underlying boost graph
 ***********************************************************/

typedef boost::adjacency_list<boost::multisetS, boost::listS, boost::undirectedS, msg::Node, msg::Edge> Graph;
typedef boost::graph_traits<Graph>::edge_descriptor GraphEdge;
typedef boost::graph_traits<Graph>::vertex_descriptor GraphVertex;


/************************************************************
 * Other useful definitions
 ***********************************************************/

typedef std::map<unsigned, tf::Pose> NodePoseMap;
typedef std::set<unsigned> NodeSet;
typedef std::set<unsigned> EdgeSet;

/*********************************************************//*
 * Represents a graph where nodes represent poses,
 * edges represent constraints between them.  Nodes and edges
 * have (independent) ids which are unsigned integers, and
 * edges have associated constraints, which consist of a
 * relative pose (of the destination w.r.t the source), and
 * precision matrix.
 *
 * Represents the actual graph using a boost graph library
 * adjacency_list. You can use (non-modifying) algorithms from
 * the BGL on the underlying graph.  Get the graph using the
 * graph method, and convert node/edge ids to BGL descriptors
 * using idVertex and idEdge.
 ***********************************************************/
class ConstraintGraph
{
public:

  /****************************************
   * Construction
   ****************************************/

  /// Create empty graph
  ConstraintGraph ();

  /// Copy constructor
  ConstraintGraph (const ConstraintGraph& g);

  /// Assignment operator
  virtual ConstraintGraph& operator= (const ConstraintGraph& g);

  /// Destructor
  virtual ~ConstraintGraph ();

  /****************************************
   * Basic modification
   ****************************************/
  
  /// Add a new node, and return its id, which is > 0.
  unsigned addNode ();

  /// \brief Add a new node with a given id
  /// \throw DuplicateNodeIdException if already exists
  void addNode (unsigned id);

  /// Add a new edge and return its id
  unsigned addEdge (unsigned from, unsigned t, const PoseWithPrecision& constraint);

  /// \brief Add a new edge with the given id
  /// \throw DuplicateEdgeIdException if already exists
  void addEdge (unsigned id, unsigned from, unsigned t, const PoseWithPrecision& constraint);


  /****************************************
   * Basic const ops
   ****************************************/

  /// \brief Get set of node ids
  NodeSet allNodes() const;

  /// \brief Get set of edge ids
  EdgeSet allEdges() const;

  /// \brief Get set of edges incident to a node
  /// \note Returned value may change; make a copy if necessary
  EdgeSet incidentEdges (unsigned n) const;

  /// \brief Get pair of nodes adjacent to this edge, in order (src, dest)
  /// \throw UnknownEdgeIdException
  std::pair<unsigned, unsigned> incidentNodes (unsigned e) const;

  /// \brief Get constraint attached to this edge
  /// \note Returned value may change; make a copy if necessary
  const PoseWithPrecision& getConstraint (unsigned e) const;
  
  /// \retval Does a node with this id exist?
  bool nodeIdExists (unsigned n) const;

  /// \retval Does an edge with this id exist?
  bool edgeIdExists (unsigned e) const;


  /****************************************
   * Ops on underlying graph
   ****************************************/

  /// \brief Return (a const reference to) the underlying Boost graph.
  ///
  /// To get convert ids into edge/vertex handles for use with the returned object,
  /// use idVertex and idEdge 
  const Graph& graph () const;

  /// \brief Get the node handle given an id
  /// \throw UnknownNodeIdException
  GraphVertex idVertex (unsigned n) const;

  /// \brief Get the edge handle given an id
  GraphEdge idEdge (unsigned e) const;

  /****************************************
   * Optimized poses
   ****************************************/

  /// \brief Get optimized pose of a node, if it exists
  /// \throw UnknownNodeIdException
  /// \throw NoOptimizedPoseExecption
  tf::Pose getOptimizedPose (unsigned n) const;

  /// Does this node have an optimized pose?
  /// \throw UnknownIdException
  bool hasOptimizedPose (unsigned n) const;

  /// \brief Set optimized pose in graph
  /// \throw UnknownNodeIdException
  /// \note It's the caller's responsibility to ensure that the pose being set here is consistent
  /// with all the poses already in the graph; if not, use setOptimizedPoses
  void setOptimizedPose (unsigned n, const tf::Pose& pose);

  /// \brief Replace the set of optimized poses currently stored in graph with new ones
  /// \throw UnknownNodeIdException
  void setOptimizedPoses (const NodePoseMap& poses);
  
private:

  /****************************************
   * Modifying utilities
   ****************************************/

  void initializeFrom (const ConstraintGraph& other);
  
  unsigned next_node_id_;
  unsigned next_edge_id_;
  std::map<unsigned, GraphVertex> vertex_map_;
  std::map<unsigned, GraphEdge> edge_map_;
  Graph graph_;
};

} // namespace

#endif // include guard
