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
 * Implementation of diff_publisher.h
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/diff_publisher.h>
#include <pose_graph/message_conversion.h>

namespace pose_graph
{

namespace msg=graph_mapping_msgs;

DiffPublisher::DiffPublisher () :
  diff_pub_(nh_.advertise<msg::ConstraintGraphDiff>("graph_diffs", 100, true)),
  graph_srv_(nh_.advertiseService("get_full_graph", &DiffPublisher::getGraph, this)),
  current_id_(1)
{}


void DiffPublisher::addDiff (msg::ConstraintGraphDiff* diff)
{
  boost::mutex::scoped_lock lock(mutex_);
  diff->id = current_id_++;
  applyDiff(&graph_, *diff);
  diff_pub_.publish(*diff);
}

bool DiffPublisher::getGraph (msg::GetGraph::Request& req, msg::GetGraph::Response& resp)
{
  boost::mutex::scoped_lock lock(mutex_);
  resp.graph = constraintGraphToMessage(graph_);
  resp.id = current_id_-1;
  return true;
}

void DiffPublisher::setGraph (const ConstraintGraph& graph)
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    graph_ = graph;
    current_id_ += 2; // make all subscribers think they dropped a diff
  }
  // It's fine if a call to getGraph intervenes here, but not addDiff
  msg::ConstraintGraphDiff fake_diff;
  addDiff(&fake_diff); // Trigger all subscribers requesting new graph
}

 


} // namespace
