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
 * Defines DiffPublisher class, which publishes incremental diffs to a constraint graph
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_DIFF_PUBLISHER_H
#define POSE_GRAPH_DIFF_PUBLISHER_H

#include <pose_graph/constraint_graph.h>
#include <graph_mapping_msgs/GetGraph.h>
#include <graph_mapping_msgs/ConstraintGraphDiff.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace pose_graph
{

class DiffPublisher
{
public:

  /// \brief Constructor that initializes empty graph
  DiffPublisher ();

  /// \brief add the next diff
  /// \effect Stamp diff with current id
  void addDiff (graph_mapping_msgs::ConstraintGraphDiff* diff);

  /// Allow subscribers to request the entire graph
  bool getGraph (graph_mapping_msgs::GetGraph::Request& req, graph_mapping_msgs::GetGraph::Response& resp);

  /// Reset the entire graph; this will trigger all subscribers requesting a new full graph
  /// Caller responsible for making sure this isn't interleaved with a call to addDiff
  void setGraph (const ConstraintGraph& graph);

private:

  /****************************************
   * Associated objects
   ****************************************/

  mutable boost::mutex mutex_;
  ros::NodeHandle nh_;
  ros::Publisher diff_pub_;
  ros::ServiceServer graph_srv_;

  /****************************************
   * State
   ****************************************/

  ConstraintGraph graph_;
  unsigned current_id_;
  

};

} // namespace

#endif // include guard
