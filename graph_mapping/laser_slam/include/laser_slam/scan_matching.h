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
 * Functions to do with laser scan matching
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_SCAN_MATCHING_H
#define LASER_SLAM_SCAN_MATCHING_H

#include <laser_slam/LocalizedScan.h>
#include <laser_slam/util.h>
#include <karto_scan_matcher/karto_scan_matcher.h>
#include <pose_graph/constraint_graph.h>
#include <pose_graph/graph_db.h>

namespace laser_slam
{

typedef boost::shared_ptr<karto_scan_matcher::KartoScanMatcher> MatcherPtr;

karto_scan_matcher::ScanMatchResult scanMatchNodes (const pose_graph::ConstraintGraph& g, MatcherPtr matcher, 
                                                    const pose_graph::NodeSet& nodes, const ScanMap& scans,
                                                    const sensor_msgs::LaserScan& scan,
                                                    const tf::Pose& init_estimate, const tf::Pose& laser_offset);


/// Basically, do repeated local matches by discretizing the world using global_resolution
karto_scan_matcher::ScanMatchResult globalLocalization (const pose_graph::ConstraintGraph& g, MatcherPtr local_matcher,
                                                        const pose_graph::NodeSet& nodes, const ScanMap& scans,
                                                        const sensor_msgs::LaserScan& scan, const tf::Pose& laser_offset,
                                                        double global_resolution, double angular_resolution);

/// Basically, do repeated local matches by discretizing the world using global_resolution
karto_scan_matcher::ScanMatchResult globalLocalization (const pose_graph::ConstraintGraph& g, MatcherPtr local_matcher,
                                                        const pose_graph::NodeSet& nodes, const ScanMap& scans,
                                                        const sensor_msgs::LaserScan& scan, const tf::Pose& laser_offset,
                                                        double global_resolution, double angular_resolution,
                                                        double min_x, double min_y, double max_x, double max_y);

} // namespace

#endif // include guard
