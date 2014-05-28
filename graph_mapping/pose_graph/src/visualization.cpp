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
 * Implements visualization for constraint graphs
 *
 * \author Bhaskara Marthi
 */

#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/ros.h>
#include <pose_graph/visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/foreach.hpp>

namespace pose_graph
{

namespace vm=visualization_msgs;
namespace util=graph_mapping_utils;
namespace gm=geometry_msgs;

ConstraintGraphVisualizer::ConstraintGraphVisualizer (bool visualize_node_ids) :
  param_nh_("~"), graph_frame_(util::searchParam<std::string>(param_nh_, "optimization_frame", "graph_optimization")),
  visualize_node_ids_(visualize_node_ids),
  comm_nh_(), marker_pub_(comm_nh_.advertise<vm::Marker> ("visualization_marker", 10)),
  marker_array_pub_(comm_nh_.advertise<vm::MarkerArray> ("visualization_marker_array", 10))
{
}


void ConstraintGraphVisualizer::visualize (const ConstraintGraph& g) const
{
  vm::Marker edges, node_marker;
  vm::MarkerArray nodes;
  const unsigned NODE_MARKER_ID_BASE = 1000;

  edges.header.frame_id = node_marker.header.frame_id = graph_frame_;
  node_marker.id = 1;
  edges.id = 0;

  node_marker.header.stamp = edges.header.stamp = ros::Time::now();
  
  edges.ns = node_marker.ns = "constraint_graph";
  if (visualize_node_ids_)
    node_marker.type = vm::Marker::SPHERE;
  else
    node_marker.type = vm::Marker::SPHERE_LIST;
  edges.type = vm::Marker::LINE_LIST;
  edges.action = node_marker.action = vm::Marker::ADD;
  edges.scale.x = 0.03;
  node_marker.scale.x = 0.07;
  node_marker.scale.y = 0.07;
  node_marker.scale.z = 0.07;
  node_marker.pose.orientation.w = 1.0;
  edges.color.b = node_marker.color.r = 1.0;
  edges.color.a = node_marker.color.a = 1.0;

  nodes.markers.reserve(g.allNodes().size());

  BOOST_FOREACH (const unsigned n, g.allNodes()) {
    if (!g.hasOptimizedPose(n)) {
      ROS_DEBUG_STREAM_NAMED ("visualize", "Not visualizing node " << n << " as it has no optimized pose");
      continue;
    }

    // Sphere at node position
    const tf::Pose pose = g.getOptimizedPose(n);
    const gm::Pose gm_pose = util::toPose(pose);
    const unsigned node_ind = nodes.markers.size();
    if (visualize_node_ids_)
    {
      nodes.markers.push_back(node_marker);
      nodes.markers[node_ind].pose.position = gm_pose.position;
      nodes.markers[node_ind].id = n + NODE_MARKER_ID_BASE;
    }
    else
    {
      node_marker.points.push_back(gm_pose.position);
    }

    // Lines for edges
    BOOST_FOREACH (const unsigned e, g.incidentEdges(n)) {
      if (g.incidentNodes(e).first == n) {
        edges.points.push_back(gm_pose.position);
        const gm::Pose pose2 = util::transformPose(pose, g.getConstraint(e).pose);
        edges.points.push_back(pose2.position);
      }
    }
  }

  marker_pub_.publish(edges);
  if (visualize_node_ids_)
    marker_array_pub_.publish(nodes);
  else
    marker_pub_.publish(node_marker);
}

} // namespace
