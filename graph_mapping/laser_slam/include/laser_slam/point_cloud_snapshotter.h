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
 * Defines the PointCloudSnapshotter class, which saves laser scans and
 * associated information (see the LocalizedScan message) at each node of
 * the graph
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_POINT_CLOUD_SNAPSHOTTER_H
#define LASER_SLAM_POINT_CLOUD_SNAPSHOTTER_H

#include <laser_slam/RecentScanTime.h>
#include <laser_slam/LocalizedScan.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pose_graph/diff_subscriber.h>
#include <pose_graph/graph_db.h>
#include <graph_mapping_msgs/ConstraintGraphDiff.h>
#include <graph_mapping_msgs/NodePoses.h>
#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>


namespace laser_slam
{

/// \brief Collects laser scans and transforms them into a point cloud w.r.t
/// the base frame at a fixed point in time.  
/// Also publishes a 2d occupancy grid based on the scans
class PointCloudSnapshotter
{
public:

  PointCloudSnapshotter (boost::shared_ptr<tf::TransformListener> tf, 
                         ros::NodeHandle param_nh);

  /// \return A time between \a t1 and \a t2 corresponding to a recent scan, or uninitialized if no such scan exists
  /// \pre t2 >= t1
  boost::optional<ros::Time> recentScanTime (const ros::Time& t1, const ros::Time& t2) const;

private:
  
  bool recentScanTimeCallback(RecentScanTime::Request& req,
                              RecentScanTime::Response& resp);
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan);
  void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void diffCallback
  (boost::optional<const graph_mapping_msgs::ConstraintGraphDiff&> diff,
   const pose_graph::ConstraintGraph& graph);
  void locCallback (graph_mapping_msgs::LocalizationDistribution::ConstPtr m);
  void storeOptPoses (graph_mapping_msgs::NodePoses::ConstPtr m);
  void publishMap (const ros::TimerEvent& e);
  void processAndSaveScan (sensor_msgs::LaserScan::ConstPtr scan,
                           const ros::Time& t, const unsigned n);
  void processAndSaveScan (sensor_msgs::LaserScan::ConstPtr scan,
                           const geometry_msgs::PoseStamped& pos);
  void processAndSaveCloud (const unsigned n,
                            sensor_msgs::PointCloud2::ConstPtr cloud,
                            const ros::Time& t);

  laser_slam::LocalizedScan::Ptr processScan (sensor_msgs::LaserScan::ConstPtr scan,
                                              const geometry_msgs::Pose& sensor_pose) const;
  tf::Pose ffPoseAt (const ros::Time& t);

  /****************************************
   * Params
   ****************************************/
  
  ros::NodeHandle param_nh_;
  const std::string fixed_frame_, sensor_frame_, base_frame_, global_frame_;
  const ros::Duration transform_wait_duration_;
  const double grid_resolution_;
  const ros::Duration scan_save_min_gap_; // The minimum increment between successive saved scans
  geometry_msgs::Pose sensor_pose_; // This is actually const, but needs tf initialized to compute
  const double scan_time_tol_;
  const bool cleanup_grid_;
  const double robot_radius_;

  /************************************************************
   * State
   ***********************************************************/
  
  pose_graph::NodePoseMap opt_poses_;
  boost::circular_buffer<sensor_msgs::LaserScan::ConstPtr> recent_scans_;
  boost::circular_buffer<sensor_msgs::PointCloud2::ConstPtr> recent_clouds_;
  boost::optional<geometry_msgs::PoseStamped> last_loc_;
  
  // Probably don't need this separately any more given we store last_loc_
  boost::optional<unsigned> ref_node_;
  
  bool optimize_flag_;

  /****************************************
   * Associated objects
   ****************************************/

  mutable boost::mutex mutex_;
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> tf_;
  pose_graph::CachedNodeMap<laser_slam::LocalizedScan> scans_;
  const pose_graph::CachedNodeMap<geometry_msgs::Pose> ff_poses_;
  pose_graph::CachedNodeMap<sensor_msgs::PointCloud2> clouds_;
  ros::Subscriber laser_sub_, cloud_sub_, loc_sub_;
  pose_graph::DiffSubscriber diff_sub_;
  ros::Publisher map_pub_;
  ros::ServiceClient get_poses_client_;
  ros::Timer map_pub_timer_;
  ros::ServiceServer recent_scan_time_server_;
};

const double SCAN_BUFFER_SIZE=100;
const double CLOUD_BUFFER_SIZE=5;

} // namespace

#endif // include guard
