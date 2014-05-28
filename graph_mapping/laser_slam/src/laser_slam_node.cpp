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
 * Node that runs the mapper.  The only extension is to modify things
 * so that new nodes are created at time points where there's a laser scan.
 *
 * \author Bhaskara Marthi
 */



#include <laser_slam/point_cloud_snapshotter.h>
#include <laser_slam/scan_intersection.h>
#include <pose_graph/exception.h>
#include <graph_slam/graph_mapper.h>
#include <graph_mapping_utils/ros.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/msg.h>
#include <graph_mapping_utils/to_string.h>
#include <pose_graph/graph_search.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

namespace laser_slam
{

namespace msg=graph_mapping_msgs;
namespace pg=pose_graph;
namespace util=graph_mapping_utils;
namespace gm=geometry_msgs;

using std::vector;

typedef boost::optional<ros::Time> MaybeTime;
typedef boost::optional<gm::PoseStamped> MaybeLoc;

const double SCAN_TIME_INC = 0.2;

class LaserMapper : public graph_slam::GraphMapper
{
public:

  LaserMapper ();
  ~LaserMapper() {}

  // We generate new nodes at time points corresponding to scans received
  // by the point cloud snapshotter
  virtual MaybeTime newNodeTime ();

private:

  bool relativelyUnseen (const ros::Time t) const;

  ros::NodeHandle nh_, param_nh_;

  const double min_unseen_proportion_;
  
  ros::ServiceClient recent_time_client_;
  ScanIntersection scan_int_;

};


ScanIntersection makeScanInt ()
{
  vector<tf::Vector3> poly(4);
  poly[1].setX(1);
  poly[1].setY(5);
  poly[2].setX(8);
  poly[3].setX(1);
  poly[3].setY(-5);
  return ScanIntersection(poly, 1000);
}


LaserMapper::LaserMapper() :
  GraphMapper(), param_nh_("~"),
  min_unseen_proportion_(util::getParam<double>(param_nh_,
                                                "min_unseen_proportion", 0.2)),
  recent_time_client_(nh_.serviceClient<RecentScanTime>("recent_scan_time")),
  scan_int_(makeScanInt())
{
}


bool LaserMapper::relativelyUnseen (const ros::Time t) const
{
  MaybeLoc l = localizations_.lastLocalization();
  if (l)
  {
    try {
      const unsigned ref = util::refNode(*l);
    const tf::Pose ref_pose = graph_.getOptimizedPose(ref);
    const tf::Pose opt_pose = util::absolutePose(ref_pose, l->pose);
    pg::OptimizedDistancePredicate pred(graph_, opt_pose.getOrigin(),
                                        5.0);
    pg::NodeSet nodes = filterNearbyNodes(graph_, ref, pred);
    vector<tf::Pose> scan_poses;
    scan_poses.reserve(nodes.size());
    BOOST_FOREACH (const unsigned n, nodes)
      scan_poses.push_back(graph_.getOptimizedPose(n));
    const double prop = scan_int_.unseenProportion(opt_pose, scan_poses);
                     
    ROS_DEBUG_STREAM_NAMED ("add_node", "Proportion of scan that was unseen was " << prop);
    return prop > min_unseen_proportion_;
    }
    catch (pg::NoOptimizedPoseException& e)
    {
      ROS_INFO_STREAM ("Not adding node because of exception: " << e.what());
      return false;
    }
  }
  else
  {
    ROS_INFO ("Not checking scan unseen-ness because have no localization");
    return true;
  }
}


MaybeTime LaserMapper::newNodeTime ()
{
  MaybeTime t;
  RecentScanTime srv;
  const ros::Time now = ros::Time::now();
  srv.request.t1 = now-ros::Duration(SCAN_TIME_INC);
  srv.request.t2 = now;
  if (recent_time_client_.call(srv)) 
  {
    if (srv.response.found && relativelyUnseen(srv.response.t))
      t = srv.response.t;
  }
  else
    ROS_ERROR ("Service call to recent_scan_time failed.  Not saving a scan.");
  return t;
}


}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "laser_slam_node");

  // We need to start ros threads already since the initialization of the objects uses callbacks
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Set up the node and spin
  laser_slam::LaserMapper node;
  ros::spin();
  return 0;
}
