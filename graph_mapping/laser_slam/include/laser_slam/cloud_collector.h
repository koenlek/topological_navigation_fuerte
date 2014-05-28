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
 * Filter that collects multiple clouds into a group before publishing them
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_COLLECT_CLOUDS_H
#define LASER_SLAM_COLLECT_CLOUDS_H

#include <laser_slam/CloudCollectorConfig.h>
#include <pcl_ros/filters/filter.h>
#include <tf/transform_listener.h>

namespace laser_slam
{

/// \brief PCL filter that collects clouds
class CloudCollector: public pcl::Filter<sensor_msgs::PointCloud2>
{
  using pcl::Filter<sensor_msgs::PointCloud2>::filter_name_;

public:

  /// \brief Empty constructor
  CloudCollector (unsigned clouds_per_group = 1u) :
    clouds_per_group_(clouds_per_group)
  {
    filter_name_ = "CloudCollector";
  }

private:

  friend class CloudCollectorROS;

  /// \brief Whenever we have these many clouds, we'll batch them and publish a single output cloud
  int clouds_per_group_;

  /// \brief Fixed frame into which clouds are transformed when received
  std::string fixed_frame_;

  /// \brief Frame in which output clouds are published
  std::string base_frame_;

  /// \brief Stored clouds
  std::vector<sensor_msgs::PointCloud2> stored_clouds_;

  pcl::PointCloud<pcl::PointXYZ> cloud_;
  unsigned num_clouds_;

  tf::TransformListener tf_;

  /// \brief Apply the filter
  /// \param output the resultant point cloud message
  virtual void applyFilter (sensor_msgs::PointCloud2& output);

};


/// \brief ROS wrapper for the pcl filter
class CloudCollectorROS : public pcl_ros::Filter
{
protected:

  virtual void onInit ();

  inline void
  filter (const PointCloud2::ConstPtr& input, const IndicesConstPtr& indices,
          PointCloud2& output)
{
  impl_.setInputCloud(input);
  impl_.setIndices(indices);
  impl_.filter(output);
}


  
  virtual inline bool child_init (ros::NodeHandle& nh)
  {
    return true;
  }

private:

  CloudCollector impl_;
  
};


} // namespace

#endif // include guard
