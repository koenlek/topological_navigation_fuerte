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
 * Code for visualizing roadmaps, and converting to and from ros messages
 *
 * \author Bhaskara Marthi
 */

#ifndef TOPOLOGICAL_ROADMAP_ROS_CONVERSION_H
#define TOPOLOGICAL_ROADMAP_ROS_CONVERSION_H

#include <topological_roadmap/roadmap.h>
#include <topological_nav_msgs/TopologicalRoadmap.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace topological_roadmap
{

namespace msg=topological_nav_msgs;

/// \brief Convert Roadmap into TopologicalRoadmap ROS message
msg::TopologicalRoadmap::Ptr toRosMessage (const Roadmap& r);

/// \brief Convert TopologicalRoadmapROS message back into ROS
Roadmap fromRosMessage (const msg::TopologicalRoadmap& r);


/// Visualize roadmap \a r using \a pub, which must be a publisher of
/// VisualizationMarker messages.
/// \param frame Frame to transform data into before publishing.
/// \param use_node_ids If true, each node will be a separate marker with the
/// corresponding node's id, to facilitate debugging.  If false, the entire
/// roadmap will be published as a single marker.  
void visualize (const Roadmap& r, ros::Publisher& pub,
                tf::TransformListener& tf, const std::string& frame,
                bool use_node_ids, const std::vector<unsigned>& path);


} // namespace

#endif // include guard
