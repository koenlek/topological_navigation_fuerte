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
 * A simple class for keeping track of displacement in a fixed frame
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_SLAM_ODOMETER_H
#define GRAPH_SLAM_ODOMETER_H

#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace graph_slam
{

/// \brief Keeps track of displacement of a moving frame w.r.t fixed frame
/// 
/// Create an instance, then call getDisplacement() to get the
/// current displacement wrt the reference pose.  Use reset() to reset the reference
/// pose.
///
/// Unlike tf, this doesn't have a fixed time window
class Odometer
{
public:

  /// \brief Create, and reset the reference pose to the current one
  /// \param tf A pointer to an existing tf transform listener
  /// The pointed-to TransformListener must stay alive for the lifetime of
  /// this object
  /// \param timeout Wait this long for the initial transform
  /// \pre ros::init has been called in this process
  Odometer (tf::TransformListener* tf, const std::string& base_frame,
            const std::string& fixed_frame, const ros::Duration& timeout = ros::Duration(0.0));

  /// \brief Reset the reference pose the one at time \a t
  /// \throws tf::TransformException
  void reset (const ros::Time& t, const ros::Duration& timeout = ros::Duration(0.0));

  /// \brief Reset the reference pose to the most recent one
  /// \throws tf::TransformException
  ros::Time reset (const ros::Duration& timeout = ros::Duration(0.0));

  /// \brief Get the relative displacement at time \a t
  /// \throws tf::TransformException
  /// \return The pose at time t w.r.t the reference pose
  tf::Pose getDisplacement (const ros::Time& t = ros::Time());

private:

  const ros::Duration wait_inc_; // How often status messages get printed when waiting
  
  tf::TransformListener* tf_;
  std::string base_frame_;
  std::string fixed_frame_;
  tf::Stamped<tf::Pose> reference_pose_;
  
};


} // namespace

#endif // include guard
