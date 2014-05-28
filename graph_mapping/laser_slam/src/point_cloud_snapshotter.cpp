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

#include <laser_slam/util.h>
#include <laser_slam/global_map.h>
#include <laser_slam/point_cloud_snapshotter.h>
#include <laser_slam/exception.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/exceptions.h>
#include <pcl_ros/transforms.h>
#include <pose_graph/graph_search.h>
#include <graph_mapping_utils/utils.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>



namespace laser_slam
{

namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace util=graph_mapping_utils;
namespace pg=pose_graph;
namespace msg=graph_mapping_msgs;
namespace nm=nav_msgs;

using std::string;
using boost::circular_buffer;

typedef boost::mutex::scoped_lock Lock;
typedef boost::optional<ros::Time> MaybeTime;

const double SCAN_TIMEOUT=30.0;
const double SCAN_UPDATE_DISTANCE_THRESHOLD=1.0;
const double SCAN_UPDATE_ANGLE_THRESHOLD=0.5;


PointCloudSnapshotter::PointCloudSnapshotter
(boost::shared_ptr<tf::TransformListener> tf, ros::NodeHandle param_nh) :
  param_nh_(param_nh),
  fixed_frame_(util::searchParam<string>(param_nh_, "fixed_frame")),
  sensor_frame_(util::searchParam<string>(param_nh_, "laser_frame")),
  base_frame_(util::searchParam<string>(param_nh_, "base_frame")),
  global_frame_(util::searchParam<string>(param_nh_, "optimization_frame")),
  transform_wait_duration_(0.2),
  grid_resolution_(util::getParam<double>(param_nh_, "grid_resolution")),
  scan_save_min_gap_(util::getParam<double>(param_nh_,
                                            "scan_save_min_gap", 0.1)),
  scan_time_tol_(util::getParam<double>(param_nh_, "scan_time_tolerance")),
  cleanup_grid_(util::getParam<bool>(param_nh_, "cleanup_grid", true)),
  robot_radius_(util::getParam<double>(param_nh_, "robot_radius", -1)),
  recent_scans_(SCAN_BUFFER_SIZE), recent_clouds_(CLOUD_BUFFER_SIZE),
  optimize_flag_(true), nh_(), tf_(tf), 
  scans_("graph_mapping", "scans"), ff_poses_("graph_mapping", "fixed_frame_poses"),
  clouds_("graph_mapping", "clouds"),
  laser_sub_(nh_.subscribe("scan", 1,
                           &PointCloudSnapshotter::scanCallback, this)),
  cloud_sub_(nh_.subscribe("cloud", 1,
                           &PointCloudSnapshotter::cloudCallback, this)),
  loc_sub_(nh_.subscribe("graph_localization", 5,
                         &PointCloudSnapshotter::locCallback, this)),
  diff_sub_(boost::bind(&PointCloudSnapshotter::diffCallback, this, _1, _2)),
  map_pub_(nh_.advertise<nm::OccupancyGrid>("global_map", 5)),
  get_poses_client_(nh_.serviceClient<msg::GetPoses>("get_node_poses")),
  map_pub_timer_(util::timerFromRate(nh_, util::getParam<double>
                                     (param_nh_, "grid_pub_rate", 0.2),
                                     &PointCloudSnapshotter::publishMap, this)),
  recent_scan_time_server_(nh_.advertiseService
                           ("recent_scan_time",
                            &PointCloudSnapshotter::recentScanTimeCallback,
                            this))
{
}


// Functor used for binary search in scan/cloud buffer
// Will work for any message type M that has a header
template <class M>
struct BufferOrdering
{
  bool operator() (const typename M::ConstPtr& m,
                   const ros::Time& t)
  {
    return (m->header.stamp<t);
  }
};


// Binary search in a scan/cloud buffer for a message
// (nonstrictly) between t1 and t2
template <class M>
MaybeTime binarySearchMsg (const ros::Time& t1,
                           const ros::Time& t2,
                           const circular_buffer<typename M::ConstPtr>& buf)
{
  ROS_ASSERT (t2>=t1);
  typename circular_buffer<typename M::ConstPtr>::const_iterator pos =
    lower_bound(buf.begin(), buf.end(), t2,
                BufferOrdering<M>());
  MaybeTime t;
  if (pos!=buf.begin()) {
    typename M::ConstPtr m = *(pos-1);
    if (m->header.stamp >= t1)
      t = m->header.stamp;
  }
  ROS_ASSERT (!t || (*t >= t1 && *t <= t2));
  return t;
}

MaybeTime PointCloudSnapshotter::recentScanTime(const ros::Time& t1,
                                                const ros::Time& t2) const
{
  Lock lock(mutex_);
  return binarySearchMsg<sm::LaserScan>(t1, t2, recent_scans_);
}



bool PointCloudSnapshotter::recentScanTimeCallback
(RecentScanTime::Request& req, RecentScanTime::Response& resp)
{
  MaybeTime mt = recentScanTime(req.t1, req.t2);
  if (mt) 
  {
    resp.t = *mt;
    resp.found = true;
  }
  else 
  {
    resp.found = false;
  }
  
  return true;
}

/// Save the scan in the buffer, so long as it's been sufficiently long
/// since the previous one
void PointCloudSnapshotter::scanCallback (const sm::LaserScan::ConstPtr& scan)
{
  Lock lock(mutex_);
  if (recent_scans_.empty() || (*(--recent_scans_.end()))->header.stamp +
      scan_save_min_gap_ < scan->header.stamp)
  {
    recent_scans_.push_back(scan);

    // Also, if this scan is much newer than the one currently stored
    // with ref node, replace it
    /*
    if (ref_node_)
    {
      try
      {
        LocalizedScan::ConstPtr old_scan = scans_.get(*ref_node_);
        const double disp = util::length(last_loc_->pose.position);
        const double angle = tf::getYaw(last_loc_->pose.orientation);
        if (scan->header.stamp >
            old_scan->scan.header.stamp + ros::Duration(SCAN_TIMEOUT) &&
            disp<SCAN_UPDATE_DISTANCE_THRESHOLD &&
            fabs(angle)<SCAN_UPDATE_ANGLE_THRESHOLD)
        {
          
          // processAndSaveScan(scan, *last_loc_);
          // Skipping scan update for now
        } 
      }
      catch (pg::DataNotFoundException& e)
      {
        ROS_WARN ("Skipping scan age check as scan not found for %u", *ref_node_);
      }
    }
    */
  }
}

/// Save point cloud in buffer
void PointCloudSnapshotter::cloudCallback (const sm::PointCloud2::ConstPtr& cloud)
{
  Lock l(mutex_);
  //if (recent_clouds_.empty() || (*(--recent_clouds_.end()))->header.stamp +
  //      cloud_save_min_gap_ < cloud->header.stamp)
  recent_clouds_.push_back(cloud);
}




/// Whenever a new node is added, save a scan at the corresponding time
void PointCloudSnapshotter::diffCallback
(boost::optional<const msg::ConstraintGraphDiff&> diff,
 const pg::ConstraintGraph& graph)
{
  ROS_DEBUG_COND_NAMED (diff, "snapshot",
                        "Received diff with %zu nodes, %zu edges",
                        diff->new_nodes.size(), diff->new_edges.size());
  ROS_DEBUG_COND_NAMED (!diff, "snapshot",
                        "Received full graph with %zu nodes",
                        graph.allNodes().size());
  // Check if this is a node addition, as opposed to an edge addition
  if (diff && diff->new_nodes.size() == 1) {
    ROS_ASSERT (diff->new_node_timestamps.size() == 1 &&
                diff->new_edges.size() == 0);
    const ros::Time& t = diff->new_node_timestamps[0];
    const unsigned n(diff->new_nodes[0].id);
    Lock lock(mutex_);

    // Find the closest time, verify it's close enough
    circular_buffer<sm::LaserScan::ConstPtr>::const_iterator pos =
      lower_bound(recent_scans_.begin(), recent_scans_.end(),
                  t, BufferOrdering<sm::LaserScan>());
    if (pos==recent_scans_.end() && !recent_scans_.empty()) pos--;
    if (pos==recent_scans_.end() ||
        fabs((*pos)->header.stamp.toSec()-t.toSec()) > scan_time_tol_) {
      ROS_WARN_STREAM_NAMED ("snapshot",
                             "Couldn't find buffered data with stamp close to "
                             << t << "; not saving a scan.  Currently have " <<
                             recent_scans_.size() << " scans.");
      ROS_WARN_STREAM_COND_NAMED (pos!=recent_scans_.end(), "snapshot",
                                  "  Closest scan was at " <<
                                  (*pos)->header.stamp <<
                                  " and scan time tolerance parameter was "
                                  << scan_time_tol_);
      return;
    }

    // Now process and save the LocalizedScan and cloud
    processAndSaveScan(*pos, t, n);
    if (!recent_clouds_.empty())
      processAndSaveCloud(n, recent_clouds_.back(), t);
  }
  else if (!diff || diff->new_edges.size() > 0) {
    optimize_flag_ = true;
  }
}

tf::Pose PointCloudSnapshotter::ffPoseAt (const ros::Time& t)
{
  tf::StampedTransform tr;
  tf_->lookupTransform(fixed_frame_, base_frame_, t, tr);
  return tr;
}



LocalizedScan::Ptr PointCloudSnapshotter::processScan
(sm::LaserScan::ConstPtr scan, const gm::Pose& sensor_pose) const
{
  LocalizedScan::Ptr sensor_frame_cloud = boost::make_shared<LocalizedScan>();

  // 1. Transform the scan to a point cloud in the fixed frame
  sm::PointCloud fixed_frame_cloud;
  laser_geometry::LaserProjection projector_;
  projector_.transformLaserScanToPointCloud(fixed_frame_, *scan, fixed_frame_cloud, *tf_);

  // 2. Transform the fixed frame cloud to the sensor frame at the start time instant of the scan
  tf_->transformPointCloud(sensor_frame_, scan->header.stamp, fixed_frame_cloud,
                           fixed_frame_, sensor_frame_cloud->cloud);
  sensor_frame_cloud->sensor_pose = sensor_pose;
  sensor_frame_cloud->header.frame_id = "unused";
  sensor_frame_cloud->header.stamp = ros::Time(42.0); // Unused
  sensor_frame_cloud->scan = *scan;
  const gm::Point b = util::barycenter(sensor_frame_cloud->cloud);
  sensor_frame_cloud->barycenter =
    util::transformPoint(util::toPose(sensor_pose), b);

  return sensor_frame_cloud;
}


/// Process scan into a LocalizedScan relative to pose at time \a t and save it
/// with node \a n
void PointCloudSnapshotter::processAndSaveScan (sm::LaserScan::ConstPtr scan,
                                                const ros::Time& t,
                                                const unsigned n)
{
  const ros::Time start = scan->header.stamp;
  const ros::Duration scan_duration(scan->ranges.size()*scan->time_increment);
  const ros::Time end = start + scan_duration;
  
  // 0. Verify all transforms exist (can get rid of this once we have tf2)
  if (!tf_->waitForTransform(fixed_frame_, sensor_frame_, start, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, sensor_frame_, start, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, start, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, start, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, sensor_frame_, end, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, sensor_frame_, end, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, end, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, end, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, t, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, t, transform_wait_duration_);

  // Transform sensor pose at scan time into base frame at time t
  gm::PoseStamped id, scan_sensor_pose;
  id.pose.orientation.w = 1.0;
  id.header.stamp = scan->header.stamp;
  id.header.frame_id = sensor_frame_;
  tf_->transformPose(base_frame_, t, id, fixed_frame_, scan_sensor_pose);

  LocalizedScan::Ptr loc_scan = processScan(scan, scan_sensor_pose.pose);
  scans_.set(n, loc_scan);
  ROS_DEBUG_STREAM_NAMED ("snapshot", "Scan saved");
}

/// Given a pointcloud2 (presumably from a 3d sensor) in the sensor frame,
/// transform into the base frame at time \a node_time and save it
/// Doesn't take into account the fact that the base is moving
/// while the scan is taken (see processAndSaveScan)
void
PointCloudSnapshotter::processAndSaveCloud (const unsigned n,
                                            sm::PointCloud2::ConstPtr cloud,
                                            const ros::Time& node_time)
{
  ROS_DEBUG_NAMED ("snapshot", "Processing cloud of size %u, %u ",
                   cloud->height, cloud->width);
  sm::PointCloud2 transformed_cloud;
  tf::StampedTransform trans;
  string cloud_frame = cloud->header.frame_id;
  ros::Time t = cloud->header.stamp;
  try {
    
    tf_->waitForTransform(base_frame_, node_time, cloud_frame, t,
                          fixed_frame_, ros::Duration(0.5));
    tf_->lookupTransform(base_frame_, node_time, cloud_frame, t,
                         fixed_frame_, trans);
    pcl_ros::transformPointCloud(base_frame_, trans, *cloud, transformed_cloud);
    clouds_.set(n, transformed_cloud);
    ROS_DEBUG_NAMED ("snapshot", "Cloud of size %u, %u saved",
                     transformed_cloud.height, transformed_cloud.width);
  }
  catch (tf::TransformException& e) {
    ROS_DEBUG_STREAM_NAMED ("snapshot", "Not saving cloud at " << n <<
                            " due to transform timeout");
  }
}


void PointCloudSnapshotter::processAndSaveScan (sm::LaserScan::ConstPtr scan,
                                                const gm::PoseStamped& pos)
{
  const unsigned n = util::refNode(pos);
  const ros::Time start = scan->header.stamp;
  const ros::Duration scan_duration(scan->ranges.size()*scan->time_increment);
  const ros::Time end = start + scan_duration;
  
  // 0. Verify all transforms exist (can get rid of this once we have tf2)
  if (!tf_->waitForTransform(fixed_frame_, sensor_frame_, start, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, sensor_frame_, start, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, start, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, start, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, sensor_frame_, end, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, sensor_frame_, end, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, end, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, end, transform_wait_duration_);
  if (!tf_->waitForTransform(fixed_frame_, base_frame_, pos.header.stamp, transform_wait_duration_))
    throw SnapshotTransformException(fixed_frame_, base_frame_, pos.header.stamp, transform_wait_duration_);

  // Transform sensor pose at scan time into frame of ref node
  gm::PoseStamped id, scan_sensor_pose;
  id.pose.orientation.w = 1.0;
  id.header.stamp = scan->header.stamp;
  id.header.frame_id = sensor_frame_;
  tf_->transformPose(base_frame_, pos.header.stamp, id, fixed_frame_, scan_sensor_pose);
  const gm::Pose pose_in_node_frame =
    util::transformPose(util::toPose(pos.pose), scan_sensor_pose.pose);
  LocalizedScan::Ptr loc_scan = processScan(scan, pose_in_node_frame);
  scans_.set(n, loc_scan);

  using util::toString;
  ROS_INFO_STREAM ("Scan sensor pose in base frame was " << toString(scan_sensor_pose.pose)
                   << ", localization was " << util::toString(pos.pose) << ", and pose in node frame was "
                   << toString(pose_in_node_frame));
  
  
  ROS_WARN_STREAM_NAMED ("snapshot", "Scan updated");
  
}

void PointCloudSnapshotter::locCallback (msg::LocalizationDistribution::ConstPtr m)
{
  Lock l(mutex_);
  if (m->samples.size()==1) {
    last_loc_ = m->samples[0];
    ref_node_ = util::refNode(m->samples[0]);
  }
  else
    ROS_WARN_NAMED ("snapshot", "Ignoring localization message with %zu samples",
                    m->samples.size());
}

void PointCloudSnapshotter::publishMap (const ros::TimerEvent& e)
{
  // Don't do any work if there are no subscribers to the map
  if (!map_pub_.getNumSubscribers())
    return;
  
  pg::ConstraintGraph g = diff_sub_.getCurrentGraph();
  if (g.allNodes().empty())
    return;
  pg::NodeSet comp;
  bool need_to_optimize = false;
  {
    Lock l(mutex_);
    if (!ref_node_)
    {
      comp = largestComp(g);
      need_to_optimize = true;
    }
    else {
      comp = pg::componentContaining(g, *ref_node_);
      need_to_optimize = optimize_flag_ || !util::contains(opt_poses_, *ref_node_);
    }
  }
  if (need_to_optimize)
  {
    optimizeGraph(&g, &opt_poses_, comp, get_poses_client_);
    ROS_DEBUG_STREAM_NAMED ("snapshot_optimize", "Reoptimizing component containing "
                            << (ref_node_ ? *ref_node_ : *comp.begin()));
    optimize_flag_ = false;
  }
  g.setOptimizedPoses(opt_poses_);
  ROS_DEBUG_STREAM_NAMED ("publish_grid", "Publishing grid based on component containing " <<
                          (ref_node_ ? *ref_node_ : *comp.begin()) << " with " << comp.size() << " nodes");
  const nm::OccupancyGrid::ConstPtr map =
    generateGlobalMap(g, scans_, grid_resolution_, global_frame_, cleanup_grid_,
                      opt_poses_, robot_radius_);
  map_pub_.publish(map);
}


} // namespace laser_slam
