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
 * Definition of OdomConstraints
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_ODOM_CONSTRAINTS_H
#define LASER_SLAM_ODOM_CONSTRAINTS_H

#include <laser_slam/utils.h>
#include <graph_mapping_msgs/LocalizationDistribution.h>

namespace laser_slam
{

/// Generates just odometric constraints between successive nodes
class OdomConstraints
{
public:
  OdomConstraints (ros::NodeHandle param_nh, TfPtr tf, DbPtr db);

private:

  void diffCallback (graph_mapping_msgs::ConstraintGraphDiff::ConstPtr diff);
  void localizationCallback (graph_mapping_msgs::LocalizationDistribution::ConstPtr l);

  ros::NodeHandle param_nh_;
  const std::string fixed_frame_;
  boost::optional<unsigned> last_node_;

  boost::mutex mutex_;
  TfPtr tf_;
  DbPtr db_;
  ros::NodeHandle nh_;
  ros::Publisher constraint_pub_;
  ros::Subscriber diff_sub_, loc_sub_;
};

} // namespace

#endif // include guard
