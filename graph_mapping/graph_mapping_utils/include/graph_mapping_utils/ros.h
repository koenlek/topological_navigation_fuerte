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
 * Ros-related utilities
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_MAPPING_UTILS_ROS_H
#define GRAPH_MAPPING_UTILS_ROS_H

#include <ros/message_traits.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <boost/type_traits/remove_const.hpp>
#include <boost/optional.hpp>

namespace graph_mapping_utils
{

namespace gm=geometry_msgs;

using std::string;
using ros::Time;

/// Get the timestamp given a shared pointer to a ROS message
template <class MsgPtr>
ros::Time timestamp (const MsgPtr& data)
{
  typedef typename MsgPtr::element_type Msg;

  // This is pending a resolution to ros ticket #3093
  typedef typename boost::remove_const<Msg>::type MsgWithoutConst;
  
  return ros::message_traits::TimeStamp<MsgWithoutConst>::value(*data);
}


/// Search for a parameter in a namespace and its ancestors
template <class P>
P searchParam (const ros::NodeHandle& nh, const string& name)
{
  string global_name;
  bool found = nh.searchParam(name, global_name);
  const string ns = nh.getNamespace();
  ROS_ASSERT_MSG (found, "Could not find %s in any parent of %s", name.c_str(), ns.c_str());
                  
  P val;
  nh.getParam(global_name, val);
  ROS_DEBUG_STREAM_NAMED ("init", "Searched upwards for " << name << " from " << ns <<
                          "; found " << global_name << " = " << val);
  return val;
}

/// Search for a parameter in a namespace and its ancestors with default
template <class P>
P searchParam (const ros::NodeHandle& nh, const string& name, const P& default_val)
{
  string global_name;
  bool found = nh.searchParam(name, global_name);
  const string ns = nh.getNamespace();
  if (!found) {
    ROS_DEBUG_STREAM_NAMED ("init", "Using default value " << default_val << " for " << name);
    return default_val;
  }
  else {
    P val;
    nh.getParam(global_name, val);
    ROS_DEBUG_STREAM_NAMED ("init", "Searched upwards for " << name << " from " << ns <<
                            "; found " << global_name << " = " << val);
    return val;
  }
}

/// Get a parameter
template <class P>
P getParam (const ros::NodeHandle& nh, const string& name) 
{
  P val;
  bool found = nh.getParam(name, val);
  ROS_ASSERT_MSG (found, "Could not find parameter %s in %s",
                  name.c_str(), nh.getNamespace().c_str());
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val);
  return val;
}


/// Get a parameter, with default values
template <class P>
P getParam (const ros::NodeHandle& nh, const string& name, const P& default_val) 
{
  P val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          " (default was " << default_val << ")");
  return val;
}

inline
ros::Duration duration (const double rate)
{
  ROS_ASSERT_MSG (rate > 0, "A rate parameter was 0, which is not allowed.");
  return ros::Duration(1/rate);
}

inline
void doNothing (const ros::TimerEvent& e)
{
}

template <class F, class G>
ros::Timer timerFromRate(const ros::NodeHandle& nh, const double rate, const F& fn, const G& obj_ptr)
{
  if (rate > 1e-6)
    return nh.createTimer(duration(rate), fn, obj_ptr);
  else
    return nh.createTimer(ros::Duration(1.0), doNothing);
}

typedef boost::optional<tf::StampedTransform> MaybeTransform;
typedef boost::optional<gm::PoseStamped> MaybePose;

/// Does waitForTransform followed by lookupTransform with
/// appropriate error checking
inline
MaybeTransform getTransform
(const tf::Transformer& tf, const string& target_frame,
 const ros::Time& target_time, const string& source_frame,
 const ros::Time& source_time, const string& fixed_frame,
 const ros::Duration& timeout)
{
  MaybeTransform tr;
  if (tf.waitForTransform(target_frame, target_time, source_frame,
                          source_time, fixed_frame, timeout)) {
    try
    {
      tf::StampedTransform trans;
      tf.lookupTransform(target_frame, target_time, source_frame,
                         source_time, fixed_frame, trans);
      tr.reset(trans);
    }
    catch (tf::TransformException& e)
    {
      // In this case, tr is not set, so a null value will be returned
    }
  }
  return tr;
}

inline
MaybePose waitAndTransform
(const tf::TransformListener& tf, const gm::PoseStamped& pose_in,
 const string& target_frame, const ros::Time& target_time,
 const string& fixed_frame, const ros::Duration& timeout)
{
  MaybePose pose_out;
  if (tf.waitForTransform(target_frame, target_time,
                          pose_in.header.frame_id, pose_in.header.stamp,
                          fixed_frame, timeout*0.5))
  {
    try
    {
      gm::PoseStamped p;
      tf.transformPose(target_frame, target_time, pose_in,
                       fixed_frame, p);
      pose_out.reset(p);
    }
    catch (tf::TransformException& e)
    {
      // pose_out is not set, so a null value will be returned
    }
  }
  return pose_out;
}

inline
MaybeTransform getTransform
(const tf::TransformListener& tf, const string& target_frame,
 const string& source_frame, const ros::Time& t, const ros::Duration& timeout)
{
  MaybeTransform trans;
  if (tf.waitForTransform(target_frame, source_frame, t, timeout))
  {
    try 
    {
      tf::StampedTransform tr;
      tf.lookupTransform(target_frame, source_frame, t, tr);
      trans.reset(tr);
    }
    catch (tf::TransformException& e)
    {
      // trans is not set, so a null value will be returned
    }
  }
  return trans;
}


} // namespace

#endif // include guard
