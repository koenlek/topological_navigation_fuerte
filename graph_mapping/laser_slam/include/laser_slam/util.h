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
 * Utilities for laser slam code
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_UTIL_H
#define LASER_SLAM_UTIL_H

#include <graph_mapping_msgs/ConstraintGraphDiff.h>
#include <graph_mapping_msgs/GetPoses.h>
#include <graph_mapping_utils/geometry.h>
#include <laser_slam/LocalizedScan.h>
#include <pose_graph/diff_subscriber.h>
#include <pose_graph/graph_db.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/optional.hpp>

namespace laser_slam
{

namespace pg=pose_graph;
namespace msg=graph_mapping_msgs;
namespace gm=geometry_msgs;
namespace util=graph_mapping_utils;

pg::NodeSet largestComp (const pg::ConstraintGraph& g);

typedef boost::shared_ptr<tf::TransformListener> TfPtr;
typedef boost::optional<const msg::ConstraintGraphDiff&> OptionalDiff;
typedef pose_graph::CachedNodeMap<LocalizedScan> ScanMap;

void optimizeGraph (pg::ConstraintGraph* g, pg::NodePoseMap* poses,
                    const pg::NodeSet& nodes, ros::ServiceClient& srv);


struct BarycenterDistancePredicate
{
  BarycenterDistancePredicate (const pg::ConstraintGraph& g, 
                               const ScanMap& scans, const gm::Point p, const double max_dist) :
    g(&g), scans(&scans), p(&p), max_dist(max_dist)
  { }

  BarycenterDistancePredicate () : g(NULL), scans(NULL), p(NULL), max_dist(-42.42) {}

  bool operator() (const pg::GraphVertex v) const
  {
    const unsigned n(g->graph()[v].id);
    if (!g->hasOptimizedPose(n)) {
      return false;
    }
    /*
    else if (!scans->hasData(n)) {
      ROS_INFO_STREAM ("Didn't have a scan for " << n);
      return false;
      }*/
    else {
      try {
        const tf::Pose node_pose = g->getOptimizedPose(n);
        LocalizedScan::ConstPtr scan = scans->get(n);
        const gm::Point b = util::transformPoint(node_pose, scan->barycenter);
        return (util::euclideanDistance(*p, b) < max_dist);
      }
      catch (pg::DataNotFoundException& e) {
        ROS_INFO_STREAM("Didn't have scan for " << n);
        return false;
      }
    }
  }

  const pg::ConstraintGraph* g;
  const ScanMap* scans;
  const gm::Point* p;
  double max_dist;
};

} // namespace

#endif // include guard
