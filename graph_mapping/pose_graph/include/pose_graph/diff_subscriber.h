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
 * Subscribes to a stream of Constraint graph diffs, and does bookkeeping to keep things synchronized
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_DIFF_SUBSCRIBER_H
#define POSE_GRAPH_DIFF_SUBSCRIBER_H

#include <pose_graph/constraint_graph.h>
#include <graph_mapping_msgs/ConstraintGraphDiff.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/optional.hpp>

namespace pose_graph
{


/// Does the bookkeeping to subscribe to constraint graph diffs
/// It uses an id field to detect if it has dropped a diff.
/// If so, it calls the get_graph service to re-request the entire graph.
/// Calls a supplied callback with the diff (or uninitialized if this was a full graph) and full graph each time
/// a diff is received.
/// Threadsafe
class DiffSubscriber
{
public:

  typedef boost::function<void (boost::optional<const graph_mapping_msgs::ConstraintGraphDiff&>, const ConstraintGraph&)>
  DiffCallback;

  /// \brief Constructor starts with empty graph and sets up subscriptions
  DiffSubscriber (const DiffCallback& diff_cb);

  /// \brief Return copy of current graph
  ConstraintGraph getCurrentGraph () const;

private:

  /// \brief Subscription callback for diffs
  void diffCB (const graph_mapping_msgs::ConstraintGraphDiff& diff);

  /****************************************
   * Associated objects
   ****************************************/
  
  mutable boost::mutex mutex_;
  ros::NodeHandle nh_;
  DiffCallback diff_callback_;
  ros::Subscriber diff_sub_;
  ros::ServiceClient graph_client_;

  /****************************************
   * State
   ****************************************/
  
  ConstraintGraph graph_;
  unsigned last_id_;
};



} // namespace

#endif // include guard
