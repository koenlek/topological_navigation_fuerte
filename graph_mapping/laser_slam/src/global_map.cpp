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
 * Implementation of global_map.h
 *
 * \author Bhaskara Marthi
 */

#include <laser_slam/global_map.h>
#include <laser_slam/LocalizedScan.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <graph_mapping_utils/geometry.h>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>


namespace laser_slam
{

namespace pg=pose_graph;
namespace nm=nav_msgs;
namespace sm=sensor_msgs;
namespace gu=occupancy_grid_utils;
namespace util=graph_mapping_utils;

using std::min;
using std::max;
using std::string;
  


nm::MapMetaData getGridBounds (const pg::ConstraintGraph& graph, const double resolution)
{
  boost::optional<double> min_x, min_y, max_x, max_y;
  BOOST_FOREACH (unsigned n, graph.allNodes()) {
    if (!graph.hasOptimizedPose(n))
      continue;
    const tf::Pose pose = graph.getOptimizedPose(n);
    const double x = pose.getOrigin().x();
    const double y = pose.getOrigin().y();
    if (!min_x) {
      min_x = x;
      max_x = x;
      min_y = y;
      max_y = y;
    }
    else {
      min_x = min(x, *min_x);
      min_y = min(y, *min_y);
      max_x = max(x, *max_x);
      max_y = max(y, *max_y);
    }
  }

  nm::MapMetaData info;
  info.origin.orientation.w = 1.0; // Identity quaternion
  info.origin.position.x = *min_x-PADDING;
  info.origin.position.y = *min_y-PADDING;
  info.width = (*max_x-*min_x+2*PADDING)/resolution;
  info.height = (*max_y-*min_y+2*PADDING)/resolution;
  info.resolution = resolution;
  
  return info;
}


nm::OccupancyGrid::ConstPtr generateGlobalMap (const pg::ConstraintGraph& graph, const ScanMap& scans,
                                               const double resolution, const string& global_frame,
                                               const bool cleanup_grid, const pg::NodePoseMap& opt_poses_,
                                               const double robot_radius)
{
  nm::OccupancyGrid fake_grid;
  fake_grid.info = getGridBounds(graph, resolution);
  ROS_DEBUG_STREAM_NAMED ("global_map", "Generating global map with dimensions " << fake_grid.info
                          << " from graph with " << graph.allNodes().size() << " nodes");

  gu::OverlayClouds overlay = gu::createCloudOverlay(fake_grid, global_frame, 0.1, 30.0);
  BOOST_FOREACH (unsigned n, util::sampleSubset(graph.allNodes(), 200)) {
    if (!graph.hasOptimizedPose(n)) {
      ROS_DEBUG_STREAM_NAMED ("global_map_internal", "Omitting node " << n <<
                              " from global overlay, as it has no optimized pose");
      continue;
    }
    const tf::Pose pose = graph.getOptimizedPose(n);
    try {
      LocalizedScan::ConstPtr m = scans.get(n);
      gu::LocalizedCloud::Ptr global_frame_cloud(new gu::LocalizedCloud());
      global_frame_cloud->sensor_pose = util::transformPose(pose, m->sensor_pose);
      global_frame_cloud->header.frame_id = global_frame;
      global_frame_cloud->cloud.points = m->cloud.points;

      // Overlay onto grid
      gu::addCloud(&overlay, global_frame_cloud);
    }
    catch (pose_graph::DataNotFoundException) {
      ROS_DEBUG_STREAM_NAMED ("global_map_internal", "Omitting node " << n <<
                              " from global overlay, as it has no associated scan");
    }
  }
  if (robot_radius>0)
  {
    BOOST_FOREACH (const pg::NodePoseMap::value_type& e, opt_poses_)
    {
      gu::addKnownFreePoint(&overlay, util::toPoint(e.second.getOrigin()), robot_radius);
    }
  }

  nm::OccupancyGrid::Ptr grid = gu::getGrid(overlay);
  if (cleanup_grid) {
    gu::OverlayClouds overlay2 = gu::createCloudOverlay(*grid, global_frame, 0.1, 30.0);
    BOOST_FOREACH (unsigned n, graph.allNodes()) {
      if (!graph.hasOptimizedPose(n))
        continue;
      const tf::Pose pose = graph.getOptimizedPose(n);
      try {
        LocalizedScan::ConstPtr m = scans.get(n);
        gu::LocalizedCloud::Ptr global_frame_cloud(new gu::LocalizedCloud());
        global_frame_cloud->sensor_pose = util::transformPose(pose, m->sensor_pose);
        global_frame_cloud->header.frame_id = global_frame;
        global_frame_cloud->cloud.points = m->cloud.points;

        // Overlay onto grid
        gu::addCloud(&overlay2, global_frame_cloud);
      }
      catch (pose_graph::DataNotFoundException) {
        ROS_DEBUG_STREAM_NAMED ("global_map_internal", "Omitting node " << n <<
                                " from global overlay, as it has no associated scan");
      }
    }

    if (robot_radius>0)
    {
      BOOST_FOREACH (const pg::NodePoseMap::value_type& e, opt_poses_)
      {
        gu::addKnownFreePoint(&overlay2, util::toPoint(e.second.getOrigin()), robot_radius);
      }
    }
    nm::OccupancyGrid::Ptr grid2 = gu::getGrid(overlay2);
    grid2->header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM_NAMED ("global_map", "Done generating global map");
    return grid2;
  }
  else
    return grid;
}


sm::PointCloud2::ConstPtr generateGlobalCloud
(const pg::ConstraintGraph& graph, const CloudMap& clouds,
 const string& global_frame)
{
  ROS_DEBUG_NAMED ("global_map", "Generating global cloud");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  BOOST_FOREACH (const unsigned n, graph.allNodes()) {
    
    if (clouds.hasData(n) && graph.hasOptimizedPose(n)) {
      tf::Transform trans = graph.getOptimizedPose(n);
      pcl::PointCloud<pcl::PointXYZ> node_cloud, transformed_cloud;
      pcl::fromROSMsg(*clouds.get(n), node_cloud);
      pcl_ros::transformPointCloud(node_cloud, transformed_cloud, trans);
      ROS_DEBUG_NAMED ("global_map",
                       "Node cloud size is %zu and trans cloud size is %zu",
                       node_cloud.points.size(),
                       transformed_cloud.points.size());
      transformed_cloud.header.frame_id = global_frame;
      cloud.header.frame_id = global_frame;
      cloud += transformed_cloud;
    }
    ROS_INFO_STREAM ("Node " << n << " " << clouds.hasData(n) << " " <<
                     graph.hasOptimizedPose(n));
  }

  sm::PointCloud2::Ptr output(new sm::PointCloud2());
  pcl::toROSMsg(cloud, *output);
  ROS_DEBUG_NAMED ("global_map",
                   "Done generating global cloud with frame %s and size %u",
                   output->header.frame_id.c_str(),
                   output->height*output->width);
  return output;
}


} // namespace
