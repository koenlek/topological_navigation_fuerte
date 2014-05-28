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
 */


#ifndef POSE_GRAPH_EXCEPTION_H
#define POSE_GRAPH_EXCEPTION_H

#include <boost/format.hpp>
#include <stdexcept>
#include <string>

namespace pose_graph
{

using boost::format;
using std::string;

/// A base class for all pose_graph exceptions; provides a handy boost::format parent constructor
class PoseGraphException: public std::logic_error
{
public:
  PoseGraphException (const format& error_string) : std::logic_error(error_string.str()) {};
  PoseGraphException (const char* str) : std::logic_error(str) {};
};


/// Exception for unknown NodeId
struct UnknownNodeIdException: public PoseGraphException
{
  UnknownNodeIdException (const unsigned id): 
    PoseGraphException(format("Unknown node id %1%") % id), id(id) {}
  const unsigned id;
};

/// Exception for unknown EdgeId
struct UnknownEdgeIdException: public PoseGraphException
{
  UnknownEdgeIdException (const unsigned id): 
    PoseGraphException(format("Unknown edge id %1%") % id), id(id) {}
  const unsigned id;
};


/// Exception for trying to add a NodeId that already exists
struct DuplicateNodeIdException: public PoseGraphException
{
  DuplicateNodeIdException (const unsigned id):
    PoseGraphException(format("Node %1% already exists") % id), id(id) {}
  const unsigned id;
};


/// Exception for trying to add an EdgeId that already exists
struct DuplicateEdgeIdException: public PoseGraphException
{
  DuplicateEdgeIdException (const unsigned id):
    PoseGraphException(format("Edge %1% already exists") % id), id(id) {}
  const unsigned id;
};

/// \brief Exception for when there's no optimized pose associated with a graph node
struct NoOptimizedPoseException: public PoseGraphException
{
  NoOptimizedPoseException (const unsigned id) : PoseGraphException (format ("No optimized pose for %1%") % id) {}
};

/// \brief Topic doesn't exist in the graph db
struct UnknownDBTopicException: public PoseGraphException
{
  UnknownDBTopicException (const string& topic) : PoseGraphException (format ("Topic %1% does not exist") % topic) {}
};      

/// \brief Could not find exactly one entry for a given topic-node pair
struct DataNotFoundException: public PoseGraphException
{
  DataNotFoundException (const unsigned n, const unsigned num_entries) :
    PoseGraphException (format ("Found %1% entries for %2% instead of 1") % num_entries % n) {}
};

/// \brief Node \a n already had data
struct NodeDuplicateDataException: public PoseGraphException
{
  NodeDuplicateDataException (const unsigned n) : PoseGraphException (format ("Node %1% already had stored data") % n) {}
};      

/// \brief Exception when trying to optimize a disconnected graph
struct DisconnectedComponentException: public PoseGraphException
{
  DisconnectedComponentException (const unsigned n, const unsigned n2) :
    PoseGraphException (format ("Nodes %1% and %2% are not in the same component, so can't jointly optimize")
                        % n % n2), n(n), n2(n2)    
  {}

  const unsigned n, n2;
  
};

/// \brief Exception when no path found in graph between two nodes
struct NoPathException: public PoseGraphException
{
  NoPathException (const unsigned n, const unsigned n2) :
    PoseGraphException (format ("No path found from %1% to %2%") % n % n2) {}
};      
    
} // namespace

#endif // POSE_GRAPH_EXCEPTION_H
