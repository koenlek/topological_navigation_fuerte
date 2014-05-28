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
 * Implementation of msg.h
 *
 * \author Bhaskara Marthi
 */

#include <graph_mapping_utils/msg.h>
#include <ros/assert.h>
#include <graph_mapping_utils/geometry.h>
#include <boost/foreach.hpp>

namespace graph_mapping_utils
{

NodePoseMap fromMsg (const NodePoses& poses)
{
  ROS_ASSERT (poses.nodes.size() == poses.poses.size());
  NodePoseMap m;
  for (unsigned i=0; i<poses.nodes.size(); i++)
    m[poses.nodes[i]] = toPose(poses.poses[i]);
  return m;
}

NodePoses::ConstPtr toMsg (const NodePoseMap& poses)
{
  NodePoses::Ptr m(new NodePoses());
  BOOST_FOREACH (const NodePoseMap::value_type& e, poses) {
    m->poses.push_back(toPose(e.second));
    m->nodes.push_back(e.first);
  }
  return m;
}

} // namespace
