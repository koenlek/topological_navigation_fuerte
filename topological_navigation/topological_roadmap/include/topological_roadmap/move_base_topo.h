/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#ifndef TOPOLOGICAL_ROADMAP_MOVE_BASE_TOPO_H_
#define TOPOLOGICAL_ROADMAP_MOVE_BASE_TOPO_H_

#include <ros/ros.h>
#include <topological_nav_msgs/TopologicalRoadmap.h>
#include <topological_nav_msgs/TopologicalGraph.h>
#include <nav_msgs/GetPlan.h>

#include <topological_roadmap/roadmap.h>
#include <topological_roadmap/shortest_paths.h>

#include <boost/thread.hpp>

#include <topological_nav_msgs/MoveBaseTopoAction.h>
#include <actionlib/server/simple_action_server.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <navfn/navfn_ros.h>

#include <boost/foreach.hpp>

#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mongo_ros/message_collection.h>
#include <std_msgs/String.h>

namespace topological_roadmap
{

typedef std::vector<unsigned> WaypointVec;

struct BlockedEdge
{
  unsigned from, to;
  ros::Time blocked_time;

  BlockedEdge(unsigned from, unsigned to, ros::Time blocked_time):
    from(from), to(to), blocked_time(blocked_time){}
};

class MoveBaseTopo
{
public:
  MoveBaseTopo();;

  void roadmapCB(topological_nav_msgs::TopologicalRoadmap::ConstPtr roadmap);
  void localizationCB(const geometry_msgs::PoseStamped& l);
  void executeCB(const topological_nav_msgs::MoveBaseTopoGoal::ConstPtr& goal);
  double shortestPathDistance(const geometry_msgs::Pose& p1);
  geometry_msgs::PoseStamped nodePoseOnGrid(unsigned n, unsigned g) const;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

private:
  void cleanup();
  unsigned bestGrid (const WaypointVec& path) const;
  bool robotWithinTolerance(double tolerance_dist, unsigned node_id);
  unsigned lastWaypointOnGridIndex(const Path& plan, const std::string& global_frame);
  unsigned lastWaypointOnGridIndex(const WaypointVec& plan, const unsigned g) const;
  bool makeRoadmapPlan(const topological_nav_msgs::MoveBaseTopoGoal::ConstPtr& goal,
                       boost::optional<Path>& plan, std::string& error_string);
  void checkEdgeTimeouts(const ros::TimerEvent& e);
  void removeBlockedEdges(const std::list<BlockedEdge>& blocked_edges, Roadmap& roadmap);
  void gridUpdateCB (const std_msgs::String& m);
  nav_msgs::OccupancyGrid::ConstPtr getGrid (const unsigned g) const;
  void graphCB (topological_nav_msgs::TopologicalGraph::ConstPtr m);

  inline double squareDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    return (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x) 
      + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
  }

  tf::TransformListener tf_;

  mongo_ros::MessageCollection<nav_msgs::OccupancyGrid> grids_;
  ros::Subscriber roadmap_sub_, localization_sub_, grid_update_sub_, tmap_sub_;
  ros::Publisher path_pub_;
  ros::ServiceClient switch_grid_client_;
  mutable boost::mutex mutex_; // Mutable because const functions need to acquire lock
  Roadmap roadmap_;
  topological_map_2d::TopologicalMap tmap_;
  actionlib::SimpleActionServer<topological_nav_msgs::MoveBaseTopoAction> as_;
  boost::optional<unsigned> current_grid_;
  costmap_2d::Costmap2DROS costmap_;
  mutable navfn::NavfnROS planner_; // Mutable because it doesn't have a const-correct api
  MoveBaseClient move_base_client_;
  double progress_check_frequency_, next_waypoint_distance_;
  bool grid_switch_;
  std::string tf_prefix_;
  ros::Time last_valid_plan_;
  double planner_patience_;
  std::list<BlockedEdge> blocked_edges_;
  ros::Timer edge_watchdog_;
  double blocked_edge_timeout_;
  ros::ServiceClient make_plan_client_;
};
};

#endif // include guard
