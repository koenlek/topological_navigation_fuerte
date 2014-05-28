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
 * Implementation of scan_matching.h
 *
 * \author Bhaskara Marthi
 */

#include <laser_slam/scan_matching.h>
#include <graph_mapping_utils/geometry.h>
#include <pose_graph/exception.h>
#include <graph_mapping_utils/to_string.h>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>


namespace laser_slam
{

namespace msg=graph_mapping_msgs;
namespace ksm=karto_scan_matcher;
namespace util=graph_mapping_utils;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace pg=pose_graph;

typedef std::vector<ksm::ScanWithPose> ScanVec;

ScanVec makeRefScans(const pg::ConstraintGraph& g, const ScanMap& scans, const pg::NodeSet& nodes,
                     const tf::Pose& laser_offset)
{
  ScanVec v;
  const tf::Pose base_in_laser_frame = laser_offset.inverse();
  BOOST_FOREACH (const unsigned n, nodes) {
    try {
      ROS_ASSERT (g.hasOptimizedPose(n));
      LocalizedScan::ConstPtr loc_scan = scans.get(n);

      // The LocalizedScan stores where the laser was w.r.t. the node
      // But the karto scan matcher wants the position of the base for each scan
      const tf::Pose laser_pose = util::absolutePose(g.getOptimizedPose(n), loc_scan->sensor_pose);
      const tf::Pose base_pose = laser_pose*base_in_laser_frame;
      
      v.push_back(ksm::ScanWithPose(loc_scan->scan, util::projectToPose2D(base_pose)));
    }
    catch (pg::DataNotFoundException& e) {
      ROS_WARN_STREAM ("Didn't find scan for node " << n << "; skipping");
    }
  }
  return v;
}




ksm::ScanMatchResult scanMatchNodes (const pg::ConstraintGraph& g, MatcherPtr matcher, const pg::NodeSet& nodes,
                                     const ScanMap& scans, const sm::LaserScan& scan, const tf::Pose& init_estimate,
                                     const tf::Pose& laser_offset)
{
  const ScanVec ref_scans = makeRefScans(g, scans, nodes, laser_offset);
  ROS_DEBUG_STREAM_NAMED ("scan_matching", "Scan matching using " << ref_scans.size() << " reference scans");
  ROS_WARN_STREAM_COND (nodes.size() > ref_scans.size(), "Skipped " << nodes.size() - ref_scans.size() <<
                        " nodes due to lack of optimized poses");
  ksm::ScanMatchResult res = matcher->scanMatch(scan, util::projectToPose2D(util::toPose(init_estimate)), ref_scans);
  return res;
}



karto_scan_matcher::ScanMatchResult globalLocalization (const pose_graph::ConstraintGraph& g, MatcherPtr local_matcher,
                                                        const pg::NodeSet& nodes, const ScanMap& scans,
                                                        const sm::LaserScan& scan, const tf::Pose& laser_offset,
                                                        double global_resolution, double angular_resolution)
{
  using util::toString;
  boost::optional<double> min_x, min_y, max_x, max_y;
  ROS_ASSERT(!nodes.empty());
  BOOST_FOREACH (const unsigned& n, nodes) {
    if (g.hasOptimizedPose(n)) {
      tf::Pose p = g.getOptimizedPose(n);
      const double x = p.getOrigin().x();
      const double y = p.getOrigin().y();
      if (!min_x || x<min_x)
        min_x = x;
      if (!min_y || y<min_y)
        min_y = y;
      if (!max_x || x>max_x)
        max_x = x;
      if (!max_y || y>max_y)
        max_y = y;
    }
  }
  ROS_ASSERT_MSG(min_x, "No optimized poses found in global location");
  ROS_DEBUG_NAMED ("global_matching", "Performing global matching between (%.2f, %.2f) and (%.2f, %.2f)",
                   *min_x, *min_y, *max_x, *max_y);
  return globalLocalization(g, local_matcher, nodes, scans, scan, laser_offset, global_resolution,
                            angular_resolution, *min_x, *min_y, *max_x, *max_y);
}


karto_scan_matcher::ScanMatchResult globalLocalization (const pose_graph::ConstraintGraph& g, MatcherPtr local_matcher,
                                                        const pg::NodeSet& nodes, const ScanMap& scans,
                                                        const sm::LaserScan& scan, const tf::Pose& laser_offset,
                                                        double global_resolution, double angular_resolution,
                                                        const double min_x, const double min_y,
                                                        const double max_x, const double max_y)
{
  const ScanVec ref_scans = makeRefScans(g, scans, nodes, laser_offset);
  boost::optional<ksm::ScanMatchResult> best;
  double best_response=-42;
  const double nx = 1+(max_x-min_x)/global_resolution;
  const double ny = 1+(max_y-min_y)/global_resolution;
  const double nt = 1+6.28/angular_resolution;
  const double num_iters = nx*ny*nt;
  unsigned count=0;
  for (double x=min_x; x<=max_x; x+=global_resolution) {
    for (double y=min_y; y<=max_y; y+=global_resolution) {
      for (double theta=0; theta<6.28; theta += angular_resolution) {
        {
          count++;
          ROS_INFO_THROTTLE (1.0, "Performing global localization.  %.2f%% done.", 100.0*count/num_iters);
        }
        const gm::Pose2D p = util::makePose2D(x, y, theta);
        ksm::ScanMatchResult res =  local_matcher->scanMatch(scan, p, ref_scans);
        if (!best || res.response > best_response) {
          ROS_DEBUG_STREAM_NAMED ("global_matching", "Result " << util::toString(res.pose) <<
                                  " with response " << res.response << " is a new best");
          best = res;
          best_response = res.response;
        }
      }
    }
  }

  ROS_ASSERT(best);
  ROS_DEBUG_STREAM_NAMED ("global_matching", "Final result is " << best->pose <<
                          " with response " << best->response);
  return *best;
}


} // namespace
