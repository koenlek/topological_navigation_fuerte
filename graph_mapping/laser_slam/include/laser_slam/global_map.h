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
 * Generate a global occupancy grid given optimized poses and pose graph with laser data
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_GLOBAL_MAP_H
#define LASER_SLAM_GLOBAL_MAP_H

#include <pose_graph/constraint_graph.h>
#include <pose_graph/graph_db.h>
#include <laser_slam/LocalizedScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

namespace laser_slam
{

/// We'll create the grid with this much padding around the provided poses 
const double PADDING=10.0;

typedef pose_graph::CachedNodeMap<laser_slam::LocalizedScan> ScanMap;
typedef pose_graph::CachedNodeMap<sensor_msgs::PointCloud2> CloudMap;

/// Generate a global grid corresponding to \a graph
nav_msgs::OccupancyGrid::ConstPtr generateGlobalMap
(const pose_graph::ConstraintGraph& graph, const ScanMap& db,
 const double resolution, const std::string& global_frame, bool cleanup_grid,
 const pose_graph::NodePoseMap& poses, const double robot_radius);

/// Generate a global cloud from a graph and saved node clouds
sensor_msgs::PointCloud2::ConstPtr generateGlobalCloud
(const pose_graph::ConstraintGraph& graph, const CloudMap& db,
 const std::string& global_frame);
 




} // namespace

#endif // include guard
