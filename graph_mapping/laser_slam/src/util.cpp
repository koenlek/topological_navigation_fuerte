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
 * Implementation of util.h
 *
 * \author Bhaskara Marthi
 */

#include <laser_slam/util.h>
#include <pose_graph/graph_search.h>
#include <graph_mapping_utils/geometry.h>
#include <boost/foreach.hpp>

namespace laser_slam
{

namespace util=graph_mapping_utils;

pg::NodeSet largestComp (const pg::ConstraintGraph& g)
{
  typedef std::vector<pg::NodeSet> Comps;
  Comps comps = connectedComponents(g);
  int best = -1;
  int best_comp=-1;
  for (int c=0; c<(int)comps.size(); c++) {
    if ((int)comps[c].size() > best) {
      best_comp = c;
      best = comps[c].size();
    }      
  }
  ROS_ASSERT(best_comp>=0);
  return comps[best_comp];
}



void optimizeGraph (pg::ConstraintGraph* g, pg::NodePoseMap* poses,
                    const pg::NodeSet& nodes, ros::ServiceClient& client)
{
  msg::GetPoses srv;
  BOOST_FOREACH (const unsigned n, nodes)
    srv.request.nodes.push_back(n);
  if (!client.call(srv))
    ROS_ASSERT_MSG (false, "Service call to optimizer failed");
  poses->clear();
  for (unsigned i=0; i<srv.request.nodes.size(); i++)
    (*poses)[srv.request.nodes[i]] =
      util::toPose(srv.response.poses[i]);
  g->setOptimizedPoses(*poses);
}


} // namespace
