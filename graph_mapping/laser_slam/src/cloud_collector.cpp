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
 * Filter that collects multiple clouds into a fixed frame
 *
 * \author Bhaskara Marthi
 */

#include <pluginlib/class_list_macros.h>
#include <laser_slam/cloud_collector.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/transforms.h>

typedef laser_slam::CloudCollectorROS CloudCollectorROS;
PLUGINLIB_DECLARE_CLASS(laser_slam, CloudCollectorROS,
                        CloudCollectorROS, nodelet::Nodelet);

namespace laser_slam
{

namespace sm=sensor_msgs;
namespace dr=dynamic_reconfigure;

using std::string;

const ros::Duration TRANSFORM_TOLERANCE(0.3);

void CloudCollectorROS::onInit ()
{
  ROS_INFO ("here");
  pcl_ros::Filter::onInit();
  ros::NodeHandle nh = getPrivateNodeHandle();
  nh.param("clouds_per_group", impl_.clouds_per_group_, 20);
  nh.param("fixed_frame", impl_.fixed_frame_, string("odom_combined"));
  nh.param("base_frame", impl_.base_frame_, string("base_footprint"));
  ROS_INFO_STREAM ("Initialized cloud collector with " << impl_.clouds_per_group_
                   << " clouds per group, fixed frame " << impl_.fixed_frame_
                   << " and base frame " << impl_.base_frame_);
}


void 
CloudCollector::applyFilter (sm::PointCloud2& output)
{
  // Convert to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> input_cloud, transformed_cloud;
  pcl::fromROSMsg(*input_, input_cloud);

  // Transform to fixed frame
  if (!tf_.waitForTransform(fixed_frame_, input_cloud.header.frame_id,
                            input_cloud.header.stamp, ros::Duration(0.1)) ||
      !pcl_ros::transformPointCloud(fixed_frame_, input_cloud, transformed_cloud, tf_)) {
    ROS_INFO ("Not adding cloud since transform was unavailable");
    return;
  }

  // Add to the collected cloud
  cloud_.header.frame_id = fixed_frame_;
  cloud_ += transformed_cloud;

  // If we have enough clouds, transform to base frame and publish by setting output
  if (num_clouds_++ > (unsigned) clouds_per_group_) {
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    ros::Time now = ros::Time::now();
    if (!tf_.waitForTransform(base_frame_, fixed_frame_, now, ros::Duration(0.1)) ||
        !pcl_ros::transformPointCloud(base_frame_, cloud_, output_cloud, tf_)) {
      ROS_INFO ("Not publishing combined cloud due to lack of transform");
      return;
    }
    pcl::toROSMsg(output_cloud, output);
    cloud_ = pcl::PointCloud<pcl::PointXYZ>();
    num_clouds_ = 0;
  }
}



}
