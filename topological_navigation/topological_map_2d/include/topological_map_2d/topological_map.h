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
 * Defines the TopologicalMap class, which represents a set of 
 * overlapping local occupancy grids 
 *
 * \author Bhaskara Marthi
 */

#ifndef TOPOLOGICAL_MAP_2D_TOPOLOGICAL_MAP_H
#define TOPOLOGICAL_MAP_2D_TOPOLOGICAL_MAP_H

#include <topological_nav_msgs/TopologicalMapNode.h>
#include <topological_nav_msgs/TopologicalMapEdge.h>
#include <map>

#define BOOST_NO_HASH
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace topological_map_2d
{

namespace msg=topological_nav_msgs;

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS,
                              msg::TopologicalMapNode,
                              msg::TopologicalMapEdge> Graph;
typedef boost::graph_traits<Graph>::edge_descriptor GraphEdge;
typedef boost::graph_traits<Graph>::vertex_descriptor GraphVertex;


/// Basically a boost adjacency_list, together with additional indexes that map
/// from unsigned integers to boost node and edge descriptors (which are void
/// pointers, and therefore can't be used when serializing the message).
/// Adding and removing nodes and edges should be done with the member
/// functions below so the indexes are updated appropriately.  For everything
/// else, use the boost adjacency_list ops, though some of them are wrapped
/// for convenience, such as node/edgeInfo.
class TopologicalMap : public Graph
{
public:

  typedef msg::TopologicalMapNode Node;
  typedef msg::TopologicalMapEdge Edge;

  /// \brief Default constructor creates empty graph
  TopologicalMap ();

  /// \brief Copy constructor
  TopologicalMap (const TopologicalMap& g);

  /// \brief Assignment operator
  TopologicalMap& operator= (const TopologicalMap& g);

  /// \brief Get the node descriptor for this id.
  /// \throws UnknownNodeIdException
  GraphVertex node (unsigned id) const;

  /// \brief Get the edge descriptor for this id.
  /// \throws UnknownEdgeIdException
  GraphEdge edge (unsigned id) const;

  /// \brief Convenience function to get node info
  /// \throws UnknownNodeIdException
  Node& nodeInfo (unsigned id);

  /// \brief Convenience function to get edge info
  /// \throws UnknownEdgeIdException
  Edge& edgeInfo (unsigned id);

  /// \brief Convenience function to get node info
  /// \throws UnknownNodeIdException
  const Node& nodeInfo (unsigned id) const;

  /// \brief Convenience function to get edge info
  /// \throws UnknownEdgeIdException
  const Edge& edgeInfo (unsigned id) const;

  /// \brief Add a node.
  /// \retval Descriptor for the newly added node
  /// \throws DuplicateNodeIdException
  /// \throws InvalidNodeIdException if info.id is 0
  GraphVertex addNode (const Node& info);

  /// \brief Add an edge.
  /// \retval Descriptor for the newly added edge
  /// \throws DuplicateEdgeIdException
  /// \throws InvalidEdgeIdException if info.id is 0
  /// \throws UnknownNodeIdException
  GraphEdge addEdge (const Edge& info);

  /// \brief Remove a node
  /// \todo Not implemented
  /// \throws UnknownNodeIdException
  void removeNode (unsigned id);
  
  /// \brief Remove an edge
  /// \todo Not implemented
  /// \throws UnknownEdgeIdException
  void removeEdge (unsigned id);

private:

  /// \brief Recompute the indices from the graph
  void recomputeIndices ();

  std::map<unsigned, GraphVertex> vertex_map_;
  std::map<unsigned, GraphEdge> edge_map_;

};

/// \retval The name of the coordinate frame for grid \a g
std::string gridFrame (const unsigned g);

/// \retval The grid id corresponding to a frame of the form "grid42"
/// \throw GridFrameNameException if frame isn't of the expected form
unsigned frameGrid (const std::string& frame);

} // namespace

#endif // include guard
