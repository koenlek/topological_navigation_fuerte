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
 * Implementation of roadmap.h
 *
 * \author Bhaskara Marthi
 */

#include <topological_roadmap/roadmap.h>
#include <topological_roadmap/exception.h>
#include <graph_mapping_utils/general.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/to_string.h>
#include <boost/foreach.hpp>

namespace topological_roadmap
{

namespace gmu=graph_mapping_utils;
namespace msg=topological_nav_msgs;
namespace tmap=topological_map_2d;
namespace gm=geometry_msgs;

using std::vector;
using std::pair;

typedef std::map<unsigned, GraphVertex> VertexMap;
typedef std::map<unsigned, GraphEdge> EdgeMap;
typedef vector<gm::Point> PointVec;
typedef vector<unsigned> Nodes;
typedef vector<unsigned> Grids;

/************************************************************
 * Initialization
 ***********************************************************/


Roadmap::Roadmap () : Graph(), next_node_id_(1), next_edge_id_(1)
{}


Roadmap::Roadmap (const Roadmap& r) :
  Graph(r), next_node_id_(r.next_node_id_), next_edge_id_(r.next_edge_id_)
{
  recomputeIndexes();
}

Roadmap& Roadmap::operator= (const Roadmap& r)
{
  Graph::operator=(r);
  next_node_id_ = r.next_node_id_;
  next_edge_id_ = r.next_edge_id_;
  recomputeIndexes();
  return *this;
}

void Roadmap::recomputeIndexes ()
{
  vertex_map_.clear();
  edge_map_.clear();
  grid_nodes_.clear();
  BOOST_FOREACH (const GraphVertex& v, vertices(*this))
  {
    const NodeInfo& info = operator[](v);
    vertex_map_[info.id] = v;

    // Note operator[] will create an entry in grid_nodes_ if necessary
    grid_nodes_[info.grid].insert(info.id);
  }
  BOOST_FOREACH (const GraphEdge& e, edges(*this)) 
    edge_map_[operator[](e).id] = e;
}

/************************************************************
 * Lookup
 ***********************************************************/

GraphVertex Roadmap::node (unsigned id) const
{
  VertexMap::const_iterator pos = vertex_map_.find(id);
  if (pos==vertex_map_.end())
    throw UnknownNodeIdException(id);
  return pos->second;
}

GraphEdge Roadmap::edge (unsigned id) const
{
  EdgeMap::const_iterator pos = edge_map_.find(id);
  if (pos==edge_map_.end())
    throw UnknownEdgeIdException(id);
  return pos->second;
}

const Roadmap::NodeInfo& Roadmap::nodeInfo (const unsigned i) const
{
  return operator[](node(i));
}

const Roadmap::EdgeInfo& Roadmap::edgeInfo (const unsigned i) const
{
  return operator[](edge(i));
}

Roadmap::Nodes Roadmap::gridNodes (const unsigned g) const
{
  if (gmu::contains(grid_nodes_, g))
    return gmu::keyValue(grid_nodes_, g);
  else
    return Nodes();  
}

/************************************************************
 * Basic modification
 ***********************************************************/

unsigned Roadmap::addNode (const NodeInfo& info_orig)
{
  NodeInfo info = info_orig;
  if (info.id == 0)
  {
    while (gmu::contains(vertex_map_, next_node_id_))
      next_node_id_++;
    info.id = next_node_id_++;
  }
  if (gmu::contains(vertex_map_, info.id))
    throw DuplicateNodeIdException(info.id);
  vertex_map_[info.id] = add_vertex(info, *this);

  // Creates an entry in grid_nodes_ if necessary
  grid_nodes_[info.grid].insert(info.id);
  
  return info.id;
}

unsigned Roadmap::addEdge (const EdgeInfo& info_orig)
{
  EdgeInfo info = info_orig;
  if (info.id == 0)
  {
    while (gmu::contains(edge_map_, next_edge_id_))
      next_edge_id_++;
    info.id = next_edge_id_++;
  }
  if (gmu::contains(edge_map_, info.id))
    throw DuplicateEdgeIdException(info.id);
  
  const GraphVertex v1 = node(info.src);
  const GraphVertex v2 = node(info.dest);
  std::pair<GraphEdge, bool> res = add_edge(v1, v2, info, *this);
  if (!res.second)
    throw ParallelEdgeException(info.src, info.dest);
  edge_map_[info.id] = res.first;
  return info.id;
}

void Roadmap::deleteEdge (const unsigned id)
{
  boost::remove_edge(edge(id), *this);
  edge_map_.erase(id);
}

/************************************************************
 * Utilities
 ***********************************************************/


gm::Point positionOnGrid (const unsigned n, const unsigned g, const Roadmap& r,
                          const tmap::TopologicalMap& tmap) 
{
  const msg::RoadmapNode& info = r.nodeInfo(n);
  const unsigned g2 = info.grid;
  if (g2==g)
  {
    return info.position;
  }
  else
  {
    const tmap::GraphVertex v = tmap.node(g);
    const tmap::GraphVertex v2 = tmap.node(g2);
    pair<tmap::GraphEdge, bool> res = edge(v, v2, tmap);
    ROS_ASSERT_MSG (res.second, "No edge between grid %u and grid %u "
                    "containing %u", g, g2, n);
    const bool flip = (tmap[res.first].dest == g);
    const tf::Pose offset = gmu::toPose(tmap[res.first].offset);
    const tf::Transform tr = flip ? offset.inverse() : offset;
    return gmu::transformPoint(tr, info.position);
  }
}

pair<Nodes, PointVec>
nodesOnGrid (const unsigned g, const Roadmap& r,
             const tmap::TopologicalMap& tmap)
{
  pair<Nodes, PointVec> res;

  // Add all nodes defined wrt this grid
  BOOST_FOREACH (const unsigned n, r.gridNodes(g))
  {
    res.first.push_back(n);
    res.second.push_back(r.nodeInfo(n).position);
  }

  const double x_size = tmap.nodeInfo(g).x_size;
  const double y_size = tmap.nodeInfo(g).y_size;

  // Also look at neighboring grids in topological graph
  BOOST_FOREACH (const tmap::GraphEdge e,
                 out_edges(tmap.node(g), tmap)) 
  {
    const bool flip = (tmap[e].dest == g);
    const tf::Pose offset = gmu::toPose(tmap[e].offset);
    const tf::Transform tr = flip ? offset.inverse() : offset;
    const unsigned g2 = flip ? tmap[e].src : tmap[e].dest;
    // Now tr maps from the neighbor grid g2 to g

    BOOST_FOREACH (const unsigned n, r.gridNodes(g2))
    {
      ROS_ASSERT(g2==r.nodeInfo(n).grid);
      const gm::Point& pos = r.nodeInfo(n).position;
      const gm::Point local_pos = gmu::transformPoint(tr, pos);
      // Now local_pos is the position of the waypoint in frame g

      ROS_DEBUG_STREAM_NAMED ("nodes_on_grid", "Node " << n << " position on " << g2 << " is "
                              << gmu::toString(pos) << " and on " << g <<
                              " is " << gmu::toString(local_pos));

      if (fabs(local_pos.x) < x_size/2 && fabs(local_pos.y) < y_size/2)
      {
        ROS_DEBUG_NAMED ("nodes_on_grid", "On grid");
        res.first.push_back(n);
        res.second.push_back(local_pos);
      }
    }
  }

  return res;
  
}



} // namespace
