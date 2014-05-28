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
 * Template function implementations for graph_search.h
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_GRAPH_SEARCH_IMPL_H
#define POSE_GRAPH_GRAPH_SEARCH_IMPL_H

#include <boost/foreach.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/breadth_first_search.hpp>


namespace pose_graph
{

template <class Predicate>
NodeSet filterNearbyNodes (const ConstraintGraph& g, const unsigned n, const Predicate& p)
{
  return filterNearbyNodes (g, n, p, AllEdges());
}

template <class G>
class Visitor : public boost::default_bfs_visitor
{
public:
  void discover_vertex (const GraphVertex& v, const G& g)
  {
    ROS_DEBUG_STREAM_NAMED ("graph_search", "Discovering vertex " << g[v].id );
  }
};

template <class NodePred, class EdgePred>
NodeSet filterNearbyNodes (const ConstraintGraph& g, const unsigned n, const NodePred& node_pred,
                           const EdgePred& edge_pred)
{
  // Set up an implicit filtered graph of all nodes satisfying the predicate
  const GraphVertex v = g.idVertex(n);
  typedef boost::filtered_graph<Graph, EdgePred, NodePred> FilteredGraph;
  const FilteredGraph filtered(g.graph(), edge_pred, node_pred);

  // Set up a color map to be used in breadth-first search
  typedef std::map<GraphVertex, boost::default_color_type> ColorMap;
  ColorMap cmap;
  boost::associative_property_map<ColorMap> color_pmap(cmap);

  // Traverse the filtered graph starting at v
  boost::breadth_first_search(filtered, v, color_map(color_pmap).visitor(Visitor<FilteredGraph>()));

  // Extract out the visited nodes
  NodeSet nodes;
  BOOST_FOREACH (GraphVertex w, vertices(filtered)) {
    ROS_ASSERT_MSG(cmap[w] != boost::gray_color, "Unexpectedly found a gray node after graph search");
    if (cmap[w] == boost::black_color)
      nodes.insert(filtered[w].id);
  }
  return nodes;  
}


} // namespace

#endif // include guard
