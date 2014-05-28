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

#include <laser_slam/point_cloud_snapshotter.h>
#include <graph_slam/graph_mapper.h>
#include <graph_mapping_utils/ros.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/msg.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

namespace util=graph_mapping_utils;
namespace msg=graph_mapping_msgs;
namespace gm=geometry_msgs;
namespace pg=pose_graph;

using std::string;
typedef boost::shared_ptr<tf::TransformListener> TfPtr;
typedef msg::ConstraintGraphDiff::ConstPtr DiffPtr;
typedef boost::mutex::scoped_lock Lock;

namespace laser_slam
{

/// Publishes odometry-based constraints and localization
class OdomSlam
{
public:
  OdomSlam (TfPtr tf);

private:

  void diffCallback (DiffPtr diff);
  void updateLocalization (const ros::TimerEvent& e);
  
  ros::NodeHandle param_nh_;
  const string fixed_frame_, opt_frame_, base_frame_;
  TfPtr tf_;
  warehouse::WarehouseClient db_;
  pg::CachedNodeMap<gm::Pose> ff_poses_;

  boost::optional<unsigned> last_node_;
  tf::Pose last_pose_;
  ros::Time last_node_stamp_;

  boost::mutex mutex_;
  ros::NodeHandle nh_;
  ros::Publisher constraint_pub_, loc_pub_;
  ros::Subscriber diff_sub_;
  ros::Timer loc_timer_;
};


OdomSlam::OdomSlam (TfPtr tf) :
  param_nh_("~"), fixed_frame_(util::getParam<string>(param_nh_, "fixed_frame")),
  opt_frame_(util::getParam<string>(param_nh_, "optimization_frame")),
  base_frame_(util::getParam<string>(param_nh_, "base_frame")), tf_(tf),
  db_("graph_mapping"), ff_poses_(&db_, "fixed_frame_poses"),
  constraint_pub_(nh_.advertise<msg::GraphConstraint>("graph_constraints", 10)),
  loc_pub_(nh_.advertise<msg::LocalizationDistribution> ("graph_localization", 5)),
  diff_sub_(nh_.subscribe("graph_diffs", 100, &OdomSlam::diffCallback, this)),
  loc_timer_(nh_.createTimer(util::duration(util::getParam<double>(param_nh_, "localization_rate", 5.0)),
                             &OdomSlam::updateLocalization, this))
{
  last_node_ = 1;
  last_pose_.setIdentity();
  last_node_stamp_ = ros::Time::now();
}

void OdomSlam::diffCallback (DiffPtr diff)
{
  Lock l(mutex_);
  if (diff->new_nodes.size() == 1) {
    const unsigned n(diff->new_nodes[0].id);
    tf::Pose pose;
    while (true) {
      try {
        pose = util::toPose(*ff_poses_.get(n));
        break;
      }
      catch (pg::DataNotFoundException& e) {
        ROS_INFO_STREAM ("Still waiting for fixed frame pose");
        ros::Duration(0.2).sleep();
      }      
    }
    if (last_node_) {
      msg::GraphConstraint::Ptr constraint(new msg::GraphConstraint());
      constraint->src = *last_node_;
      constraint->dest = n;
      constraint->constraint.pose = util::toPose(util::relativePose(pose, last_pose_));
      constraint->constraint.precision = util::makePrecisionMatrix(1, 1, 1, 1);
      constraint_pub_.publish(constraint);
    }
    last_node_ = n;
    last_pose_ = pose;
    last_node_stamp_ = diff->new_node_timestamps[0];
  }
}

void OdomSlam::updateLocalization (const ros::TimerEvent& e)
{
  Lock lock(mutex_);
  if (last_node_) {
    msg::LocalizationDistribution::Ptr d(new msg::LocalizationDistribution());
    d->stamp = ros::Time::now();
    gm::PoseStamped id;
    id.pose.orientation.w = 1.0;
    id.header.stamp = d->stamp;
    id.header.frame_id = base_frame_;
    try {
      tf_->waitForTransform(fixed_frame_, base_frame_, d->stamp, ros::Duration(0.2));
      gm::PoseStamped fixed_frame_pose;
      tf_->transformPose(fixed_frame_, id, fixed_frame_pose);
      tf::Pose ff_pose = util::toPose(fixed_frame_pose.pose);
      d->samples.resize(1);
      d->samples[0].header.frame_id = util::nodeFrame(*last_node_);
      d->samples[0].pose = util::toPose(util::relativePose(ff_pose, last_pose_));
      loc_pub_.publish(d);
    }
    catch (tf::TransformException& e) {
      ROS_WARN_STREAM ("Skipping localization due to tf exception: " << e.what());
    }
  }
  else
    ROS_DEBUG_STREAM_NAMED ("localization", "Not updating localization because have no last node");
}


}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "odom_slam_node");

  // We need to start ros threads already since the initialization of the objects uses callbacks
  ros::AsyncSpinner spinner(3);
  spinner.start();

  TfPtr tf(new tf::TransformListener());

  // The mapper main loop
  graph_slam::GraphMapper node;

  // Stores point clouds from the base laser, and publishes global occupancy grid
  laser_slam::PointCloudSnapshotter snapshotter(tf, ros::NodeHandle("~"));

  // Generates odometry based constraints and localization
  laser_slam::OdomSlam odom_slam(tf);

  ros::spin();
  return 0;
}
