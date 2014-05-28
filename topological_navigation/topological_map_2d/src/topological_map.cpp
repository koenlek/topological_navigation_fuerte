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
 * Implementation of topological_map.h
 *
 * \author Bhaskara Marthi
 */

#include <topological_map_2d/topological_map.h>
#include <topological_map_2d/exception.h>
#include <graph_mapping_utils/general.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace topological_map_2d
{

namespace util=graph_mapping_utils;
typedef std::map<unsigned, GraphVertex> VertexMap;
typedef std::map<unsigned, GraphEdge> EdgeMap;
using std::pair;

/************************************************************
 * Utility
 ***********************************************************/

string gridFrame (const unsigned g)
{
  return "grid" + boost::lexical_cast<string>(g);
}

unsigned frameGrid (const std::string& frame)
{

  size_t grid_pos = frame.find("grid");

  if (grid_pos == string::npos)
    throw GridFrameNameException(frame);

  int id = atoi(frame.c_str()+grid_pos+4);
  if (id<=0 || id==INT_MAX)
    throw GridFrameNameException(frame);

  return id;
}

/************************************************************
 * Initialization
 ***********************************************************/

TopologicalMap::TopologicalMap () :
  Graph()
{}

TopologicalMap::TopologicalMap (const TopologicalMap& g) :
  Graph(g)
{
  recomputeIndices();
}

TopologicalMap& TopologicalMap::operator= (const TopologicalMap& g)
{
  Graph::operator=(g);
  recomputeIndices();
  return *this;
}

void TopologicalMap::recomputeIndices ()
{
  vertex_map_.clear();
  edge_map_.clear();
  BOOST_FOREACH (const GraphVertex& v, vertices(*this))
    vertex_map_[operator[](v).id] = v;
  BOOST_FOREACH (const GraphEdge& e, edges(*this))
    edge_map_[operator[](e).id] = e;
}


/************************************************************
 * Lookup
 ***********************************************************/


GraphVertex TopologicalMap::node (const unsigned id) const
{
  VertexMap::const_iterator pos = vertex_map_.find(id);
  if (pos==vertex_map_.end())
    throw UnknownNodeIdException(id);
  return pos->second;
}

GraphEdge TopologicalMap::edge (const unsigned id) const
{
  EdgeMap::const_iterator pos = edge_map_.find(id);
  if (pos==edge_map_.end())
    throw UnknownEdgeIdException(id);
  return pos->second;
}

TopologicalMap::Node& TopologicalMap::nodeInfo (const unsigned id)
{
  return operator[](node(id));
}

TopologicalMap::Edge& TopologicalMap::edgeInfo (const unsigned id)
{
  return operator[](edge(id));
}

const TopologicalMap::Node& TopologicalMap::nodeInfo (const unsigned id) const
{
  return operator[](node(id));
}

const TopologicalMap::Edge& TopologicalMap::edgeInfo (const unsigned id) const
{
  return operator[](edge(id));
}


/************************************************************
 * Basic modification
 ***********************************************************/

GraphVertex TopologicalMap::addNode (const Node& info)
{
  if (info.id==0)
    throw InvalidNodeIdException(info.id);
  if (util::contains(vertex_map_, info.id))
    throw DuplicateNodeIdException(info.id);
  return (vertex_map_[info.id] = add_vertex(info, *this));
}

GraphEdge TopologicalMap::addEdge (const Edge& info)
{
  if (info.id==0)
    throw InvalidEdgeIdException(info.id);
  if (util::contains(edge_map_, info.id))
    throw DuplicateEdgeIdException(info.id);
  const GraphVertex v1 = node(info.src);
  const GraphVertex v2 = node(info.dest);
  pair<GraphEdge, bool> res = add_edge(v1, v2, info, *this);
  if (!res.second)
    throw ParallelEdgeException(info.src, info.dest);
  return (edge_map_[info.id] = res.first);
}



} // namespace
