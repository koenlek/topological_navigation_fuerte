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

#include "odometer.h"

namespace graph_slam
{

Odometer::Odometer (tf::TransformListener* tf, const std::string& base_frame,
                    const std::string& fixed_frame, const ros::Duration& timeout) :
  wait_inc_(5.0), tf_(tf), base_frame_(base_frame), fixed_frame_(fixed_frame)
{
  reset(timeout);
}

void Odometer::reset (const ros::Time& t, const ros::Duration& timeout)
{
  const ros::Duration inc = timeout < wait_inc_ ? timeout : wait_inc_;
  for (unsigned i=0; i<ceil(timeout.toSec()/inc.toSec()); i++) {
    if (tf_->waitForTransform(fixed_frame_, base_frame_, t, inc))
      break;
    ROS_INFO_STREAM ("Waiting for transform from " << base_frame_ << " to " << fixed_frame_ <<
                     " at " << t);
  }
  // If we didn't find a transform, a tf exception will be thrown on the next step
  
  tf::StampedTransform trans;
  tf_->lookupTransform(fixed_frame_, base_frame_, t, trans);
  reference_pose_.setData(trans); // Slicing happens here
  reference_pose_.stamp_ = trans.stamp_;
  reference_pose_.frame_id_ = fixed_frame_;
}

ros::Time Odometer::reset (const ros::Duration& timeout)
{
  reset (ros::Time(), timeout);
  return reference_pose_.stamp_;
}

tf::Pose Odometer::getDisplacement (const ros::Time& t)
{
  tf::StampedTransform trans;
  tf_->lookupTransform(fixed_frame_, base_frame_, t, trans);
  return reference_pose_.inverseTimes(trans);
}


} // namespace graph_slam
