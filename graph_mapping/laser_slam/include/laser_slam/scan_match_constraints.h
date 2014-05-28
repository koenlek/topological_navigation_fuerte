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
 * Definition of ScanMatchConstraints
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_SCAN_MATCH_CONSTRAINTS_H
#define LASER_SLAM_SCAN_MATCH_CONSTRAINTS__H

#include <laser_slam/util.h>
#include <laser_slam/scan_matching.h>
#include <laser_slam/LocalizedScan.h>
#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <graph_slam/localization_buffer.h>
#include <graph_mapping_msgs/NodePoses.h>
#include <pose_graph/graph_db.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/optional.hpp>
#include <std_msgs/UInt64.h>

namespace laser_slam
{

namespace msg=graph_mapping_msgs;
namespace pg=pose_graph;
namespace gm=geometry_msgs;

typedef std::vector<unsigned> Chain;
typedef std::vector<msg::GraphConstraint> ConstraintVec;

const std::string PREV_NODE_TOPIC("scan_match_constraints_prev_node");
const std::string NEXT_NODE_TOPIC("scan_match_constraints_next_node");


/// Uses scan matching to generate sequential and loop closure constraints
class ScanMatchConstraints
{
public:
  ScanMatchConstraints (ros::NodeHandle param_nh, TfPtr tf);
  

private:

  void addConstraints (OptionalDiff diff, const pg::ConstraintGraph& g);
  ConstraintVec getRunningConstraints (const unsigned n, const pg::ConstraintGraph& g,
                                       const gm::PoseStamped& l, const pg::NodeSet& running) const;
  ConstraintVec getNearLinkConstraints (const unsigned n, const pg::ConstraintGraph& g,
                                        const gm::PoseStamped& l, const pg::NodeSet& running) const;
  ConstraintVec getLoopConstraints (const unsigned n, const pg::ConstraintGraph& g,
                                    const gm::PoseStamped& l, const pg::NodeSet& running) const;
  pg::NodeSet getRunningNodes(const pg::ConstraintGraph& g, const unsigned n,
                               const gm::PoseStamped& l) const;
  msg::GraphConstraint makeConstraint (const pg::ConstraintGraph& g, const unsigned n,
                                       const karto_scan_matcher::ScanMatchResult& res,
                                       const pg::NodeSet& nodes, LocalizedScan::ConstPtr scan) const;
  msg::GraphConstraint makeConstraint (const pg::ConstraintGraph& g, const unsigned n,
                                       const unsigned ref, const karto_scan_matcher::ScanMatchResult& res,
                                       LocalizedScan::ConstPtr scan) const;
  unsigned nodeWithNearestBarycenter (const pg::ConstraintGraph& g,
                                         const pg::NodeSet& nodes, const geometry_msgs::Point& p) const;
  void locCallback (msg::LocalizationDistribution::ConstPtr m);
  boost::optional<unsigned> previousNode (const unsigned n) const;
  boost::optional<unsigned> nextNode (const unsigned n) const;
  void addSequenceLink (const unsigned n1, const unsigned n2);
  Chain getChain (const unsigned n, const pg::NodeSet& nodes,
                  const pg::NodeSet& forbidden) const;
  void updateDistanceTravelled (const ros::TimerEvent& e);
  tf::Pose fixedFramePose (const unsigned n) const;

  /****************************************
   * Parameters
   ****************************************/

  const std::string fixed_frame_;
  const std::string base_frame_;
  const std::string laser_frame_;
  const std::string opt_frame_;
  const double local_window_size_;
  const double running_scan_match_size_;
  const double running_scan_match_res_;
  const unsigned near_link_min_chain_size_;
  const double chain_distance_threshold_;
  const double loop_scan_match_size_;
  const double loop_scan_match_res_;
  const double loop_match_window_size_;
  const unsigned loop_match_min_chain_size_;
  const double min_loop_response_;
  const double max_loop_match_variance_;
  const double min_sequential_response_;
  const double max_nbd_size_;
  const unsigned running_buffer_size_;
  tf::Pose laser_offset_;

  /****************************************
   * State
   ****************************************/
  
  bool initialized_;
  boost::optional<unsigned> last_added_node_;
  double distance_since_last_added_node_;
  boost::circular_buffer<unsigned> running_nodes_;
  pg::NodePoseMap optimized_poses_;

  /****************************************
   * Associated objects
   ****************************************/

  boost::mutex mutex_;
  TfPtr tf_;
  pose_graph::CachedNodeMap<std_msgs::UInt64> before_, after_;
  ScanMap scans_;
  pose_graph::CachedNodeMap<geometry_msgs::Pose> ff_poses_;
  graph_slam::LocalizationBuffer loc_buf_;
  ros::NodeHandle nh_;
  pg::DiffSubscriber diff_sub_;
  ros::Subscriber loc_sub_;
  ros::Publisher constraint_pub_;
  ros::Publisher marker_pub_, pose_pub_;
  ros::ServiceClient get_poses_client_;
  MatcherPtr sequential_matcher_;
  MatcherPtr loop_matcher_;
  ros::Timer distance_update_timer_;
  
};

} // namespace laser_slam

#endif // include guard
