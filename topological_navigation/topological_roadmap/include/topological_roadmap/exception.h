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
 * Exceptions thrown by code in topological_roadmap package
 *
 * \author Bhaskara Marthi
 */

#ifndef TOPOLOGICAL_ROADMAP_EXCEPTION_H
#define TOPOLOGICAL_ROADMAP_EXCEPTION_H

#include <boost/format.hpp>
#include <stdexcept>

namespace topological_roadmap
{

using boost::format;
using std::string;

/// A base class for all topological_roadmap exceptions; provides a handy
/// boost::format parent constructor
class RoadmapException: public std::logic_error
{
public:
  RoadmapException (const format& error_string) : std::logic_error(error_string.str()) {};
  RoadmapException (const char* str) : std::logic_error(str) {};
};

/// Exception for unknown node id
struct UnknownNodeIdException: public RoadmapException
{
  UnknownNodeIdException (const unsigned id): 
    RoadmapException(format("Unknown node id %1%") % id), id(id) {}
  const unsigned id;
};

/// Exception for unknown EdgeId
struct UnknownEdgeIdException: public RoadmapException
{
  UnknownEdgeIdException (const unsigned id): 
    RoadmapException(format("Unknown edge id %1%") % id), id(id) {}
  const unsigned id;
};


/// Exception for trying to add a NodeId that already exists
struct DuplicateNodeIdException: public RoadmapException
{
  DuplicateNodeIdException (const unsigned id):
    RoadmapException(format("Node %1% already exists") % id), id(id) {}
  const unsigned id;
};


/// Exception for trying to add an EdgeId that already exists
struct DuplicateEdgeIdException: public RoadmapException
{
  DuplicateEdgeIdException (const unsigned id):
    RoadmapException(format("Edge %1% already exists") % id), id(id) {}
  const unsigned id;
};

/// \brief Exception when trying to add edge between two already connected nodes
struct ParallelEdgeException: public RoadmapException
{
  ParallelEdgeException (const unsigned n1, const unsigned n2) :
    RoadmapException (format ("Edge between %1% and %2% already existed")
                             % n1 % n2) {}
};      


} // namespace

#endif // include guard
