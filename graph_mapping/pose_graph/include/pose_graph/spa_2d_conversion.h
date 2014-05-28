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
 * Code to convert between PoseGraph to sba::SysSPA2D
 */

#ifndef POSE_GRAPH_SPA_2D_CONVERSION_H
#define POSE_GRAPH_SPA_2D_CONVERSION_H

#ifndef NPRINT
#define NPRINT


#include <sba/spa2d.h>
#include <pose_graph/constraint_graph.h>
#include <geometry_msgs/Pose.h>
#include <boost/shared_ptr.hpp>
#include <map>

namespace pose_graph
{

typedef boost::shared_ptr<sba::SysSPA2d> Spa2DPtr;
typedef std::map<unsigned, unsigned> NodeIndexMap;

/// Contains
/// - (shared pointer to) Spa2d object corresponding to this graph
/// - A map that says what the array index in the nodes member is corresponding to a given node id in the pose graph
struct Spa2DConversionResult
{
  Spa2DPtr spa;
  NodeIndexMap node_index_map;
  Spa2DConversionResult (const Spa2DPtr& s, const NodeIndexMap& m) : spa(s), node_index_map(m) {}
  Spa2DConversionResult ()  {} // For convenience

};

/// Return the Pose corresponding to a spa node
tf::Pose getNodePose (const sba::Node2d& n);


/// \brief Optimize a graph using SPA 2d
/// \a init Initial estimates
/// \retval map from node id to optimized pose
/// \throw DisconnectedGraphException
NodePoseMap optimizeGraph2D (const ConstraintGraph& g, const NodePoseMap& init);

/// \brief Optimize a component of a graph using SPA 2d
/// \retval map from node id to optimized pose
/// \throw DisconnectedGraphException
NodePoseMap optimizeGraph2D (const ConstraintGraph& g, const NodePoseMap& init,
                             const NodeSet& nodes);




} // namespace pose_graph

#endif // NPRINT
#endif // include guard
