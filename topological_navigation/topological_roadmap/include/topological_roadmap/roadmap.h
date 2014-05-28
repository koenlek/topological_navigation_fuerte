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
 * Defines the Roadmap class
 *
 * \author Bhaskara Marthi
 */

#ifndef TOPOLOGICAL_ROADMAP_ROADMAP_H
#define TOPOLOGICAL_ROADMAP_ROADMAP_H

#include <topological_map_2d/topological_map.h>
#include <topological_nav_msgs/RoadmapNode.h>
#include <topological_nav_msgs/RoadmapEdge.h>

namespace topological_roadmap
{

namespace msg=topological_nav_msgs;

/************************************************************
 * Underlying boost graph typedefs
 ***********************************************************/

typedef boost::adjacency_list<boost::multisetS, boost::listS, boost::undirectedS,
                              msg::RoadmapNode, msg::RoadmapEdge> Graph;
typedef boost::graph_traits<Graph>::edge_descriptor GraphEdge;
typedef boost::graph_traits<Graph>::vertex_descriptor GraphVertex;

/************************************************************
 * Class def
 ***********************************************************/

/// A topological roadmap is overlaid over a topological_map_2d::TopologicalMap
/// and used for navigation.  Each roadmap node is a navigation waypoint and
/// refers to some specific position in one of the local grids of the topological
/// map. Each edge represents a navigable path between two waypoints on the same
/// grid.  Apart from the grid wrt which it is defined, each waypoint may also
/// be on other grids, and this is what allows navigating across grids.
///
/// The class inherits from boost::adjacency_list with some additional indexing
/// information that allows nodes and edges to be referred to using integer ids
/// rather than boost graph descriptors.  To maintain this information, all
/// modifying operations on the graph should use {add,remove}{Node,Edge} rather
/// than boost::add_vertex, etc.  For other ops use the boost::adjacency_list
/// ops, though some are wrapped for convenience, e.g., node/edgeInfo.
class Roadmap : public Graph
{
public:

  typedef msg::RoadmapNode NodeInfo;
  typedef msg::RoadmapEdge EdgeInfo;
  typedef std::set<unsigned> Nodes;

  /// \brief Default constructor creates empty graph
  Roadmap ();

  /// \brief Copy constructor
  Roadmap (const Roadmap& r);

  /// \brief Assignment
  Roadmap& operator= (const Roadmap& r);

  /// \brief Get node descriptor for id
  /// \throws UnknownNodeIdException
  GraphVertex node (unsigned id) const;

  /// \brief Get edge descriptor for id
  /// \throws UnknownEdgeIdException
  GraphEdge edge (unsigned id) const;

  /// \brief Convenience function to get node info
  /// \throws UnknownNodeIdException
  const NodeInfo& nodeInfo (unsigned id) const;

  /// \brief Convenience function to get edge info
  /// \throws UnknownEdgeIdException
  const EdgeInfo& edgeInfo (unsigned id) const;

  /// \brief Add a node.  If the id of the info field is 0, 
  /// it is filled in by this function.
  /// \retval Id of newly added node.
  unsigned addNode (const NodeInfo& info);

  /// \brief Add an edge.  If the id field of the info is 0,
  /// it is filled in by this function.
  /// \retval Id of newly added edge.
  /// \throws UnknownNodeIdException
  /// \throws DuplicateEdgeIdException
  unsigned addEdge (const EdgeInfo& info);

  /// \brief Delete an edge
  /// \post There's no edge with id \a id in graph
  /// \throws UnknownEdgeIdException
  void deleteEdge (unsigned id);

  /// \retval Set of nodes defined wrt this grid.  If this 
  /// grid has never been mentioned by any node, will just
  /// return the empty set.
  Nodes gridNodes (const unsigned g) const;

  /// \brief Recompute indexing info from the underlying graph
  void recomputeIndexes ();

private:

  std::map<unsigned, GraphVertex> vertex_map_;
  std::map<unsigned, GraphEdge> edge_map_;
  std::map<unsigned, Nodes> grid_nodes_;

  // Counter used to autogenerate node ids
  unsigned next_node_id_;

  // Counter used to autogenerate edge ids
  unsigned next_edge_id_;

};


/************************************************************
 * Utilities
 ***********************************************************/

/// \retval Position of a waypoint wrt a grid on the topological map
geometry_msgs::Point
positionOnGrid (unsigned n, unsigned g, const Roadmap& r,
                const topological_map_2d::TopologicalMap& t);


/// \retval All waypoints lying on a given grid of the topological map
std::pair<std::vector<unsigned>, std::vector<geometry_msgs::Point> >
nodesOnGrid (const unsigned g, const Roadmap& r,
             const topological_map_2d::TopologicalMap& t);


} // namespace

#endif // include guard
