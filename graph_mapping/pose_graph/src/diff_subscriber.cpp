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
 * Implementation of diff_subscriber.h
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/diff_subscriber.h>
#include <graph_mapping_msgs/GetGraph.h>
#include <pose_graph/message_conversion.h>

namespace pose_graph
{

namespace msg=graph_mapping_msgs;


DiffSubscriber::DiffSubscriber (const DiffSubscriber::DiffCallback& cb) :
  diff_callback_(cb), graph_client_(nh_.serviceClient<msg::GetGraph>("get_full_graph")), last_id_(0)
{
  // We don't want to wait till the first diff is sent.  So we invoke our own callback
  // with a fake diff to trigger a service call.  Note that it's fine if 
  
  msg::ConstraintGraphDiff fake_diff;
  fake_diff.id = 42; // Anything but 1
  diffCB(fake_diff);
  
  // Now that the current graph is received, subscribe to the true diff topic
  diff_sub_ = nh_.subscribe("graph_diffs", 10, &DiffSubscriber::diffCB, this);
}


void DiffSubscriber::diffCB (const msg::ConstraintGraphDiff& diff)
{
  boost::optional<const msg::ConstraintGraphDiff&> possible_diff;
  {
    boost::mutex::scoped_lock l(mutex_);
    if (diff.id == last_id_+1) {
      applyDiff(&graph_, diff);
      possible_diff = diff;
      last_id_ = diff.id;
    }
    else {
      ROS_INFO_STREAM ("Received diff with id " << diff.id << " while last id is " << last_id_);
      msg::GetGraph srv;
      graph_client_.call(srv);
      graph_ = constraintGraphFromMessage(srv.response.graph);
      last_id_ = srv.response.id;
      ROS_INFO_STREAM ("New graph has id " << last_id_);
    }
  }
  // Not giving up the lock allows deadlock if the callback acquires a lock
  // that also surrounds other ops on this class.  It's ok to do this because
  // 1) only one instance of diffCB can be running at a time, and that's the only
  // non-const member function 2) graph_ is const for diff_callback_ 

  diff_callback_(possible_diff, graph_);
}

ConstraintGraph DiffSubscriber::getCurrentGraph () const
{
  boost::mutex::scoped_lock l(mutex_);
  return graph_;
}





} // namespace
