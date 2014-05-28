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
 * Implementation of shortest_paths.h
 *
 * \author Bhaskara Marthi
 */

#include <topological_roadmap/shortest_paths.h>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <list>

namespace topological_roadmap
{

namespace msg=topological_nav_msgs;

using std::vector;
using std::map;
using std::list;
using std::pair;
using boost::optional;

typedef map<GraphVertex, double> DistanceMap;
typedef map<GraphVertex, GraphVertex> PredMap;
typedef pair<DistanceMap, PredMap> DijkstraResult;

DijkstraResult dijkstra (const Roadmap& r, const unsigned src)
{
  using boost::associative_property_map;
  
  // Define some additional map types to store the weights and
  // indices used within Dijkstra's algorithm
  typedef map<GraphEdge, double> WeightMap;
  typedef map<GraphVertex, unsigned> IndexMap;

  // dijkstra_shortest_paths further requires std::map to be
  // wrapped in a "Property Map"
  typedef associative_property_map<DistanceMap> DistancePMap;
  typedef associative_property_map<PredMap> PredPMap;
  typedef associative_property_map<WeightMap> WeightPMap;
  typedef associative_property_map<IndexMap> IndexPMap;

  // OK, step 1: set the indices arbitrarily, because bgl won't do this for you
  IndexMap imap;
  IndexPMap index_pmap(imap);
  { 
    unsigned ind=0;
    BOOST_FOREACH (const GraphVertex v, vertices(r))
      imap[v] = ind++;
  }

  // Step 2: set the weights, using the edge lengths
  WeightMap wmap;
  WeightPMap weight_pmap(wmap);
  BOOST_FOREACH (const GraphEdge e, edges(r))
    wmap[e] = r[e].cost; 

  // Step 3: set up output maps
  DistanceMap dmap;
  DistancePMap distance_pmap(dmap);
  PredMap pmap;
  PredPMap predecessor_pmap(pmap);

  // Step 4: make the call
  boost::dijkstra_shortest_paths(r, r.node(src), weight_map(weight_pmap).\
                                 vertex_index_map(index_pmap).\
                                 distance_map(distance_pmap).\
                                 predecessor_map(predecessor_pmap));

  // Step 5: Return
  return DijkstraResult(dmap, pmap);  
}


struct SingleSourceShortestPaths
{
  unsigned src;
  GraphVertex v;
  DijkstraResult res;
  const Roadmap& r;
  SingleSourceShortestPaths (unsigned src, GraphVertex v, DijkstraResult res,
                             const Roadmap& r) :
    src(src), v(v), res(res), r(r)
  {}
};

ResultPtr shortestPaths (const Roadmap& r, const unsigned src)
{
  return boost::make_shared<SingleSourceShortestPaths>(src, r.node(src),
                                                       dijkstra(r, src), r);
}

optional<Path> extractPath (DijkstraResult& res,
                            const unsigned src,
                            const unsigned dest,
                            const Roadmap& r)
{
  BOOST_FOREACH (const PredMap::value_type& e, res.second)
  {
    const unsigned n=r[e.first].id;
    ROS_DEBUG_NAMED ("shortest_paths",
                     "Predecessor of %u is %u and cost of %u is %.2f",
                     n, r[e.second].id, n, res.first[e.first]);
  }
  optional<Path> p;
  const GraphVertex w = r.node(dest);
  if (dest == src || res.second[w]!=w)
  {
    list<unsigned> nodes;
    for (GraphVertex x=w; x!=res.second[x]; x=res.second[x]) {
      const unsigned n=r[x].id;
      nodes.push_front(n);
    }
    nodes.push_front(src);
    p = Path();
    copy(nodes.begin(), nodes.end(), back_inserter(p->first));
  }
  return p;
}

optional<Path> extractPath (ResultPtr res, const unsigned dest)
{
  return extractPath (res->res, res->src, dest, res->r);
}

} // namespace
