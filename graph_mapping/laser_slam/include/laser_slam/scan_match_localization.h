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
 * Definition of ScanMatchLocalizer class
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_SCAN_MATCH_LOCALIZATION_H
#define LASER_SLAM_SCAN_MATCH_LOCALIZATION_H

#include <laser_slam/scan_matching.h>
#include <laser_slam/LocalizedScan.h>
#include <pose_graph/graph_db.h>
#include <pose_graph/diff_subscriber.h>
#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <graph_mapping_msgs/NodePoses.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kidnapped_robot/MatchResult.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/optional.hpp>

namespace laser_slam
{

namespace msg=graph_mapping_msgs;
namespace pg=pose_graph;

/// Localization based on scan matching
class ScanMatchLocalizer
{
public:
  ScanMatchLocalizer (ros::NodeHandle param_nh,
                      boost::shared_ptr<tf::TransformListener> tf);

private:

  void update (const ros::TimerEvent& e);
  void updateUsingFixedFrame ();
  void storeScan (sensor_msgs::LaserScan::ConstPtr scan);
  void diffCB (boost::optional<const msg::ConstraintGraphDiff&> diff,
               const pg::ConstraintGraph& graph);
  void matchCB (const kidnapped_robot::MatchResult& match);
  boost::optional<tf::StampedTransform> fixedFramePoseAt (const ros::Time& t);
  void setInitialPose (const geometry_msgs::PoseWithCovarianceStamped& m);
  void recomputeOptimizedPoses (pg::ConstraintGraph* g, const pg::NodeSet& comp);
  void adjustInitialPose (const tf::Pose& p, const pose_graph::ConstraintGraph& g,
                          const pose_graph::NodeSet& comp,
                          double global_loc_radius);
  unsigned closestNode (const pg::ConstraintGraph& g,
                          const pg::NodeSet& nodes,
                          const tf::Vector3& barycenter) const;
  bool closerTo (const pg::ConstraintGraph& g,
                 const tf::Vector3& barycenter,
                 const unsigned n1, const unsigned n2) const;

  /****************************************
   * Params
   ****************************************/

  mutable boost::mutex mutex_; // Mutable because const functions can still get the lock
  ros::NodeHandle param_nh_;
  tf::Pose laser_offset_;
  const double update_rate_;
  const double scan_match_proportion_; // What proportion of updates to scan match on
  const double max_nbd_size_;
  const std::string fixed_frame_;
  const std::string base_frame_;
  const std::string opt_frame_;
  const double window_size_;
  const double odom_noise_; // For adding fake noise when debugging
  const double match_radius_; // Radius of search for scan match
  const double match_resolution_; // Resolution of search for scan match
  const double angular_resolution_; // In global matching
  const bool do_global_localization_;

  /****************************************
   * State
   ****************************************/
  
  msg::LocalizationDistribution::Ptr last_loc_;
  bool initialized_;
  sensor_msgs::LaserScan::ConstPtr last_scan_, initialization_scan_;
  
  // Mutable because getScan, which is logically const, can still modify the cache
  // This can go away once we start using an actual database in GraphDB
  pg::NodePoseMap opt_poses_;
  bool optimize_flag_;
  unsigned match_count_;
  boost::optional<ros::Time> last_match_request_time_;
  boost::optional<kidnapped_robot::MatchResult> match_result_;


  /****************************************
   * Associated objects
   ****************************************/

  boost::shared_ptr<tf::TransformListener> tf_;
  const ScanMap scans_;
  const pose_graph::CachedNodeMap<geometry_msgs::Pose> ff_poses_;
  MatcherPtr matcher_;
  ros::NodeHandle nh_;
  ros::Publisher loc_pub_, image_save_pub_, match_req_pub_;
  ros::Subscriber init_pose_sub_, scan_sub_, match_sub_;
  pg::DiffSubscriber diff_sub_;
  ros::ServiceClient get_poses_client_;
  ros::Timer update_timer_;
};

const double GLOBAL_MATCH_MIN_RESPONSE=0.6;

} // namespace

#endif // include guard
