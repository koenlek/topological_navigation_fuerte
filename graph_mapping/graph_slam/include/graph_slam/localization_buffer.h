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
 * LocalizationBuffer class holds a buffer of graph localizations
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_SLAM_LOCALIZATION_BUFFER_H
#define GRAPH_SLAM_LOCALIZATION_BUFFER_H

#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/optional.hpp>

namespace graph_slam
{

typedef boost::shared_ptr<tf::TransformListener> TfPtr;

/// \brief Maintains a history of graph localizations; uses a fixed frame to extrapolate
/// A localization is a PoseStamped whose frame id is of the form 'node42'
///
/// Threadsafe
class LocalizationBuffer
{
public:
  LocalizationBuffer (const unsigned buffer_size, const std::string& fixed_frame,
                      const std::string& base_frame, TfPtr tf, const double max_extrapolation);

  /// \throw LocalizationExtrapolationException
  geometry_msgs::PoseStamped localizationAt (const ros::Time& t) const;

  /// \throw LocalizationExtrapolationException
  geometry_msgs::PoseStamped localizationAt (const ros::Time& t,
                                                        const ros::Duration& d) const;

  /// \retval Whether we can interpolate a localization at the given time
  bool hasLocalization (const ros::Time& t) const;

  /// \return Last localization in the buffer if there's at least one, empty otherwise
  boost::optional<geometry_msgs::PoseStamped> lastLocalization () const;

private:

  void localizationCallback (graph_mapping_msgs::LocalizationDistribution::ConstPtr m);

  const std::string fixed_frame_;
  const std::string base_frame_;
  const double max_extrapolation_;

  mutable boost::mutex mutex_;
  TfPtr tf_;
  boost::circular_buffer<graph_mapping_msgs::LocalizationDistribution::ConstPtr> buffer_;
  ros::NodeHandle nh_;
  ros::Subscriber loc_sub_;
  
};

} // namespace

#endif // include guard
