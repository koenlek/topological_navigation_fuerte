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
 * Shortest paths in a roadmap
 *
 * \author Bhaskara Marthi
 */

#ifndef TOPOLOGICAL_ROADMAP_SHORTEST_PATHS_H
#define TOPOLOGICAL_ROADMAP_SHORTEST_PATHS_H

#include <topological_roadmap/roadmap.h>

namespace topological_roadmap
{

struct SingleSourceShortestPaths;
typedef boost::shared_ptr<SingleSourceShortestPaths> ResultPtr;

typedef std::vector<unsigned> NodeVec;
typedef std::vector<unsigned> EdgeVec;
typedef std::pair<NodeVec, EdgeVec> Path;

/// \brief Return object that can be used for shortest path queries
/// Note the code scales with the largest node id in the graph, and
/// assumes all edge costs are positive
ResultPtr shortestPaths (const Roadmap& r, unsigned src);

/// \brief Extract path from shortest path query, or
/// null value if there's no path
boost::optional<Path>
extractPath (ResultPtr res, unsigned dest);

} // namespace

#endif // include guard
