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
 * Implementation of graph_search.h
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/graph_search.h>
#include <pose_graph/exception.h>
#include <graph_mapping_utils/general.h>
#include <graph_mapping_utils/geometry.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <list>
#include <iostream>

namespace pose_graph
{

namespace util=graph_mapping_utils;
namespace msg=graph_mapping_msgs;
using std::map;
using std::pair;
using std::list;
using std::cout;

std::vector<NodeSet> connectedComponents (const ConstraintGraph& g)
{
  // 1. Set up color map
  typedef std::map<GraphVertex, boost::default_color_type> ColorMap;
  ColorMap cmap;
  boost::associative_property_map<ColorMap> color_pmap(cmap);
  
  // 2. Output map from vertex to component
  typedef std::map<GraphVertex, unsigned> ComponentMap;
  ComponentMap comp_map;
  boost::associative_property_map<ComponentMap> comp_pmap(comp_map);

  // 3. Connected components
  const Graph& graph = g.graph();
  const unsigned num_comps = connected_components(graph, comp_pmap, color_map(color_pmap));

  // 4. Create component sets and return
  std::vector<NodeSet> comps(num_comps);
  BOOST_FOREACH (const ComponentMap::value_type& e, comp_map) {
    comps[e.second].insert(graph[e.first].id);
  }
  return comps;
}

struct AllNodes 
{
  inline bool operator() (const GraphVertex v) const
  {
    return true;
  }
};

NodeSet componentContaining (const ConstraintGraph& g, const unsigned n)
{
  return filterNearbyNodes(g, n, AllNodes());
}


/************************************************************
 * Shortest paths and relative poses
 ***********************************************************/


typedef map<GraphVertex, double> DistanceMap;
typedef map<GraphVertex, GraphVertex> PredecessorMap;
typedef pair<DistanceMap, PredecessorMap> DijkstraResult;

DijkstraResult dijkstra (const Graph& g, const GraphVertex& src)
{
  // Define some additional map types to store the weights and indices
  // used within Dijkstra's algorithm
  typedef map<GraphEdge, double> WeightMap;
  typedef map<GraphVertex, unsigned> IndexMap;

  // dijkstra_shortest_paths further requires std::map to be
  // wrapped in a "Property Map"
  typedef boost::associative_property_map<DistanceMap> DistancePMap;
  typedef boost::associative_property_map<PredecessorMap> PredecessorPMap;
  typedef boost::associative_property_map<WeightMap> WeightPMap;
  typedef boost::associative_property_map<IndexMap> IndexPMap;

  // OK, step 1: set the indices arbitrarily, because bgl won't do this for you
  IndexMap imap;
  IndexPMap index_pmap(imap);
  { 
    unsigned ind=0;
    BOOST_FOREACH (const GraphVertex& v, vertices(g))
      imap[v] = ind++;
  }

  // Step 2: set the weights, using the edge lengths
  WeightMap wmap;
  WeightPMap weight_pmap(wmap);
  BOOST_FOREACH (const GraphEdge& e, edges(g))
  {
    const double l = util::length(g[e].constraint.constraint.pose.position);
    wmap[e] = l;
  }

  // Step 3: set up output maps
  DistanceMap dmap;
  DistancePMap distance_pmap(dmap);
  PredecessorMap pmap;
  PredecessorPMap predecessor_pmap(pmap);

  // Step 4: make the call
  boost::dijkstra_shortest_paths(g, src, weight_map(weight_pmap).vertex_index_map(index_pmap).
                                 distance_map(distance_pmap).predecessor_map(predecessor_pmap));

  // Step 5: Return
  return DijkstraResult(dmap, pmap);
}






ShortestPath shortestPath (const ConstraintGraph& g,
                           const unsigned src, const unsigned dest)
{
  typedef Graph::out_edge_iterator EdgeIter;
  const Graph& graph = g.graph();

  // Call Dijkstra's algorithm
  const GraphVertex v = g.idVertex(src);
  const GraphVertex w = g.idVertex(dest);
  const DijkstraResult res = dijkstra(graph, v);

  // Check path exists
  if (dest!=src && util::keyValue(res.second, w)==w)
    throw NoPathException(src, dest);

  // Extract node path
  list<unsigned> path;
  for (GraphVertex x=w; x!=util::keyValue(res.second, x);
       x=util::keyValue(res.second, x))
    path.push_front(graph[x].id);
  path.push_front(src);
  ShortestPath ret;
  copy(path.begin(), path.end(), back_inserter(ret.first));

  // Extract edge path
  // \todo Doesn't pick best edge when there are parallel edges
  for (unsigned i=0; i<ret.first.size()-1; i++)
  {
    const unsigned from = ret.first[i];
    const unsigned to = ret.first[i+1];
    pair<GraphEdge, bool> e = edge(g.idVertex(from), g.idVertex(to), graph);
    ROS_ASSERT(e.second);
    ret.second.push_back(graph[e.first].id);
  }
  
  return ret;
}


tf::Pose relativePose (const ConstraintGraph& g, const unsigned n1,
                       const unsigned n2)
{
  const Graph& graph = g.graph();
  ShortestPath path = shortestPath(g, n1, n2);
  tf::Pose p;
  p.setIdentity();
  BOOST_FOREACH (const unsigned e, path.second)
    p *= util::toPose(graph[g.idEdge(e)].constraint.constraint.pose);
  return p;
}



} // namespace
