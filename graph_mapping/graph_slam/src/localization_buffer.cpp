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
 * Implementation of localization_buffer.h
 *
 * \author Bhaskara Marthi
 */

#include <graph_slam/localization_buffer.h>
#include <graph_slam/exception.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/msg.h>

namespace graph_slam
{

namespace msg=graph_mapping_msgs;
namespace util=graph_mapping_utils;
namespace gm=geometry_msgs;

using std::string;

typedef msg::LocalizationDistribution::ConstPtr LocPtr;
typedef boost::mutex::scoped_lock Lock;
typedef boost::circular_buffer<LocPtr> Buf;


LocalizationBuffer::LocalizationBuffer (const unsigned size, const string& fixed_frame, const string& base_frame, TfPtr tf,
                                        const double max_extrapolation) :
  fixed_frame_(fixed_frame), base_frame_(base_frame), max_extrapolation_(max_extrapolation), tf_(tf), 
  buffer_(size), loc_sub_(nh_.subscribe("graph_localization", 5, &LocalizationBuffer::localizationCallback, this))
{}

void LocalizationBuffer::localizationCallback (LocPtr m)
{
  Lock lock(mutex_);

  // Make sure not to add out-of-order localizations
  if (!buffer_.empty()) {
    LocPtr last_loc = buffer_[buffer_.size()-1];
    const ros::Time& last_stamp = last_loc->stamp;
    if (last_stamp >= m->stamp) {
      ROS_WARN_STREAM ("Not adding localization with stamp " << m->stamp <<
                       " to buffer because last stamp was " << last_stamp);
      return;
    }
  }

  buffer_.push_back(m);
}

struct LocalizationOrdering
{
  bool operator() (const ros::Time& t, const LocPtr& l)
  {
    return (t < l->stamp);
  }
};

gm::PoseStamped LocalizationBuffer::localizationAt (const ros::Time& t) const
{
  ros::Duration d(0.0);
  return localizationAt(t, d);
}

gm::PoseStamped LocalizationBuffer::localizationAt (const ros::Time& t, const ros::Duration& timeout) const
{
  LocPtr nearest;
  tf::StampedTransform trans;
  {
    Lock lock(mutex_);
    if (buffer_.empty())
      throw LocalizationExtrapolationException(t);
  
    const Buf::const_iterator pos = upper_bound(buffer_.begin(), buffer_.end(), t, LocalizationOrdering());
    nearest = (pos == buffer_.begin()) ? *pos : *(pos-1);
    const double gap = fabs(nearest->stamp.toSec() - t.toSec());
    if (gap > max_extrapolation_)
      throw LocalizationExtrapolationException (t, nearest->stamp);

    // Figure out how far the base has moved since or until then
    tf_->waitForTransform(base_frame_, fixed_frame_, nearest->stamp, timeout);
    tf_->waitForTransform(base_frame_, fixed_frame_, t, timeout);
    tf_->lookupTransform(base_frame_, nearest->stamp, base_frame_, t, fixed_frame_, trans);
  }

  // Return corrected localization
  gm::PoseStamped l;
  l.header.frame_id =  nearest->samples[0].header.frame_id;
  l.pose = util::absolutePose(nearest->samples[0].pose, trans);
  return l;
}

bool LocalizationBuffer::hasLocalization (const ros::Time& t) const
{
  Lock l(mutex_);
  bool has_loc = false;
  if (!buffer_.empty()) {
    Buf::const_iterator pos = upper_bound(buffer_.begin(), buffer_.end(), t, LocalizationOrdering());
    LocPtr nearest = (pos == buffer_.begin()) ? *pos : *(pos-1);
    has_loc = fabs(nearest->stamp.toSec() - t.toSec()) < max_extrapolation_;
    ROS_DEBUG_STREAM_COND_NAMED (!has_loc, "localization", "Nearest localization was " << nearest->stamp);
  }
  ROS_DEBUG_STREAM_COND_NAMED (!has_loc, "localization", "No localization found at " << t
                               << "; buffer size is " << buffer_.size());
  return has_loc;
}


boost::optional<gm::PoseStamped> LocalizationBuffer::lastLocalization () const
{
  Lock lock(mutex_);
  boost::optional<gm::PoseStamped> l;
  if (!buffer_.empty()) {
    LocPtr dist = buffer_[buffer_.size()-1];
    ROS_ASSERT_MSG (dist->samples.size()==1, "Assumption of deterministic localization violated");
    l = dist->samples[0];
  }
  return l;
}

} // namespace
