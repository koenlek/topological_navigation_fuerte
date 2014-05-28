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
#include <topological_roadmap/move_base_topo.h>
#include <graph_mapping_utils/geometry.h>
#include <topological_nav_msgs/RoadmapPath.h>
#include <graph_mapping_utils/to_string.h>
#include <boost/lexical_cast.hpp>
#include <topological_map_2d/ros_conversion.h>
#include <topological_roadmap/ros_conversion.h>
#include <topological_nav_msgs/SwitchGrid.h>
#include <set>

namespace topological_roadmap

{

namespace gm=geometry_msgs;
namespace mr=mongo_ros;
namespace nm=nav_msgs;
namespace gmu=graph_mapping_utils;
namespace tmap=topological_map_2d;
namespace msg=topological_nav_msgs;

using std::vector;
using std::set;
using std::string;
using std::list;

typedef boost::mutex::scoped_lock Lock;

MoveBaseTopo::MoveBaseTopo():
  grids_("topological_navigation", "grids"), 
  as_(ros::NodeHandle(), "move_base_topo",
      boost::bind(&MoveBaseTopo::executeCB, this, _1), false),
  costmap_("costmap", tf_), move_base_client_("move_base", true),
  grid_switch_(false)
{
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.param("progress_check_frequency", progress_check_frequency_, 5.0);
  private_nh.param("next_waypoint_distance", next_waypoint_distance_, 1.0);
  private_nh.param("planner_patience", planner_patience_, 10.0);

  double block_check_period;
  private_nh.param("block_check_period", block_check_period, 1.0);
  private_nh.param("blocked_edge_timeout", blocked_edge_timeout_, 0.0);

  last_valid_plan_ = ros::Time::now();

  //get the tf prefix
  ros::NodeHandle prefix_nh;
  tf_prefix_ = tf::getPrefixParam(prefix_nh);

  planner_.initialize("planner", &costmap_);
  costmap_.stop();
  roadmap_sub_ = n.subscribe("topological_roadmap", 1,
                             &MoveBaseTopo::roadmapCB, this);
  localization_sub_ = n.subscribe("topological_localization", 1,
                                  &MoveBaseTopo::localizationCB, this);
  grid_update_sub_ = n.subscribe("warehouse/topological_navigation/grids/inserts",
                                 100, &MoveBaseTopo::gridUpdateCB, this);
  tmap_sub_ = n.subscribe("topological_graph", 1, &MoveBaseTopo::graphCB, this);
  path_pub_ = n.advertise<msg::RoadmapPath>("roadmap_path", 5);
  edge_watchdog_ = n.createTimer(ros::Duration(block_check_period),
                                 &MoveBaseTopo::checkEdgeTimeouts, this);
  make_plan_client_ =
    n.serviceClient<nm::GetPlan>("move_base_node/make_plan");
  switch_grid_client_ =
    n.serviceClient<msg::SwitchGrid>("switch_local_grid");
  as_.start();
  ROS_INFO ("move_base_topo initialization complete");
}

void MoveBaseTopo::graphCB (const msg::TopologicalGraph::ConstPtr m)
{
  Lock l(mutex_);
  tmap_ = tmap::fromMessage(*m);
}

void MoveBaseTopo::cleanup()
{
  //we'll clear our list of blocked edges when we finish a goal
  ROS_DEBUG_NAMED("exec", "Clearing blocked edges");
  blocked_edges_.clear();
  costmap_.stop();
}

gm::PoseStamped MoveBaseTopo::nodePoseOnGrid (const unsigned n, const unsigned g) const
{
  gm::PoseStamped pose;
  pose.header.frame_id = tmap::gridFrame(g);
  pose.pose.position = positionOnGrid(n, g, roadmap_, tmap_);
  pose.pose.orientation.w  = 1.0;
  return pose;
}

bool MoveBaseTopo::robotWithinTolerance(double tolerance_dist, unsigned node_id)
{
  //get the robot's position
  tf::Stamped<tf::Pose> robot_pose;
  if(!costmap_.getRobotPose(robot_pose)){
    ROS_ERROR("Cannot get the current pose of the robot, checking whether or not the robot is close enough to its next waypoint will fail. This is likely a tf problem");
    return false;
  }
  gm::PoseStamped robot_pose_msg;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

  //ok now we have the pose of the robot so we need to pose of the node that we're going to
  const unsigned g = tmap::frameGrid(costmap_.getGlobalFrameID());
  const gm::PoseStamped node_pose = nodePoseOnGrid(node_id, g);
  double sq_dist = squareDist(robot_pose_msg, node_pose);
  if(sq_dist < (tolerance_dist * tolerance_dist))
    return true;

  return false;
}

unsigned MoveBaseTopo::lastWaypointOnGridIndex (const vector<unsigned>& plan,
                                                const unsigned g) const
{
  //get the set of nodes contained in the current grid
  vector<unsigned> current_nodes_vec =
    nodesOnGrid(g, roadmap_, tmap_).first;
  set<unsigned> current_nodes(current_nodes_vec.begin(),
                              current_nodes_vec.end());

  ROS_DEBUG_STREAM_NAMED ("last_waypoint_on_grid",
                          "Finding last waypoint for grid " << g <<
                          " with nodes " << gmu::toString(current_nodes_vec));
  unsigned int furthest_valid = 0;
  for(unsigned int i = 0; i < plan.size(); ++i)
  {
    ROS_DEBUG_NAMED ("last_waypoint_on_grid", "Furthest valid is %u",
                     furthest_valid);
    //check if the node in the plan is contained within the current grid
    if(gmu::contains(current_nodes, plan[i]))
    {
      // if so, get pose of the node in the grid and its associated potential
      gm::PoseStamped node_pose = nodePoseOnGrid(plan[i], g);
      if(planner_.validPointPotential(node_pose.pose.position))  
        furthest_valid = i;
    }
    else
      //break once we get to a node that isn't contained within the current grid
      break;
  }
  ROS_DEBUG_NAMED ("last_waypoint_on_grid", "Final furthest valid is %u",
                   furthest_valid);
  return furthest_valid;
}

// Note: Assumes that the correct potential function has been computed by planner
unsigned MoveBaseTopo::lastWaypointOnGridIndex(const Path& plan,
                                               const string& global_frame)
{
  return lastWaypointOnGridIndex(plan.first, tmap::frameGrid(global_frame));
}

bool MoveBaseTopo::makeRoadmapPlan(const msg::MoveBaseTopoGoal::ConstPtr& goal,
                                   boost::optional<Path>& plan, string& error_string)
{
  plan.reset();
  unsigned best_id(0);
  double best_dist = DBL_MAX;

  //first, we need to get the pose of the robot in the frame of the grid
  tf::Stamped<tf::Pose> robot_pose;
  if(!costmap_.getRobotPose(robot_pose)){
    error_string = "Cannot navigate anywhere because the costmap could not get the current pose of the robot. Likely a tf problem.";
    return false;
  }
  else if (!current_grid_)
  {
    error_string = "Don't have a current grid yet, so can't make a plan.";
    return false;
  }
  

  gm::Point robot_point;
  robot_point.x = robot_pose.getOrigin().x();
  robot_point.y = robot_pose.getOrigin().y();

  //next, we'll compute the potential function for the current pose of the robot in the grid
  planner_.computePotential(robot_point);

  //we need to get all of the nodes that are contained within our current grids
  std::vector<unsigned> contained_nodes = nodesOnGrid(*current_grid_, roadmap_, tmap_).first;

  BOOST_FOREACH (const unsigned n, contained_nodes)
  {
    const unsigned g = tmap::frameGrid(costmap_.getGlobalFrameID());
    const gm::PoseStamped node_pose = nodePoseOnGrid(n, g);

    //pick the node with the shortest path distance to project to
    double path_dist = shortestPathDistance(node_pose.pose);
    if(path_dist < best_dist){
      best_dist = path_dist;
      best_id = n;
    }
  }

  if(best_dist == DBL_MAX)
  {
    error_string = "Cannot navigate anywhere because the robot cannot be projected onto a valid node.";
    return false;
  }

  ROS_DEBUG_STREAM("The closest node is node " << best_id <<
                   ", planning from this to node " << goal->goal_node);

  //now, we need to create a plan using the roadmap from the node to the goal node
  ResultPtr res = shortestPaths(roadmap_, best_id);
  plan = extractPath(res, goal->goal_node);
  if(!plan)
  {
    std::stringstream error_msg;
    error_msg << "Failed to find a plan between node: " << best_id << " and node: " << goal->goal_node;
    error_string = error_msg.str();
    return false;
  }

  return true;
}

void MoveBaseTopo::checkEdgeTimeouts(const ros::TimerEvent& e)
{
  Lock l(mutex_);
  //check to see what edges in our blocked list have timed out and remove them from the list
  for(list<BlockedEdge>::iterator blocked_it = blocked_edges_.begin(); blocked_it != blocked_edges_.end();){
    if(blocked_edge_timeout_ > 0.0 && blocked_it->blocked_time + ros::Duration(blocked_edge_timeout_) < ros::Time::now()){
      ROS_ERROR("Unblocking edge");
      blocked_it = blocked_edges_.erase(blocked_it);
    }
    else
      ++blocked_it;
  }
}

void MoveBaseTopo::removeBlockedEdges(const list<BlockedEdge>& blocked_edges, Roadmap& roadmap)
{
  BOOST_FOREACH (const BlockedEdge& blocked, blocked_edges)
  {
    const GraphVertex v = roadmap.node(blocked.from);
    const GraphVertex w = roadmap.node(blocked.to);
    vector<unsigned> edges_to_delete;

    // Two loops to avoid invalidating descriptors 
    BOOST_FOREACH (GraphEdge e, edge_range(v, w, roadmap))
      edges_to_delete.push_back(roadmap[e].id);
    BOOST_FOREACH (const unsigned id, edges_to_delete) 
      roadmap.deleteEdge(id);
  }
}

unsigned MoveBaseTopo::bestGrid (const vector<unsigned>& plan) const
{
  ROS_ASSERT(!plan.empty());
  const unsigned start = plan[0];
  set<unsigned> candidates;
  const unsigned g0 = roadmap_.nodeInfo(start).grid;
  candidates.insert(g0);
  BOOST_FOREACH (const tmap::GraphVertex& v,
                 adjacent_vertices(tmap_.node(g0), tmap_))
    candidates.insert(tmap_[v].id);

  unsigned best_grid=-1;
  int best_val = -1;
  BOOST_FOREACH (const unsigned g, candidates) 
  {
    const int val = lastWaypointOnGridIndex(plan, g);
    ROS_INFO ("Grid %u has %d waypoints", g, val);
    if (val > best_val)
    {
      best_val = val;
      best_grid = g;
    }
  }
  ROS_ASSERT(best_grid>0);
  return best_grid;
}

void MoveBaseTopo::executeCB(const msg::MoveBaseTopoGoal::ConstPtr& goal)
{
  ROS_DEBUG_NAMED("exec", "Received a goal, starting costmap");
  move_base_client_.waitForServer();
  costmap_.start();
  ROS_DEBUG_NAMED("exec", "costmap started");

  //we want to get a plan from the roadmap
  boost::optional<Path> plan;
  string error_string;

  unsigned int current_waypoint = 0;
  bool finished_plan = false;
  last_valid_plan_ = ros::Time::now();

  //ok... now we have a plan in the roadmap, we need to feed it to move_base
  while(ros::ok() && !finished_plan){
    move_base_msgs::MoveBaseGoal mb_goal;
    {
      //make sure that the roadmap doesn't change while we generate a goal for move_base
      Lock l(mutex_);

      //if we've just switched grids, then we need to replan
      //we'll do this everytime since we should only get close enough to our
      //exit points when  we switch grids
      ROS_DEBUG_NAMED("exec", "Planning");
      if(!makeRoadmapPlan(goal, plan, error_string)){
        if(ros::Time::now() > last_valid_plan_ + ros::Duration(planner_patience_)){
          ROS_WARN("In loop planning: %s, now: %.3f, last: %.3f, finish: %.2f", error_string.c_str(), ros::Time::now().toSec(),
                    last_valid_plan_.toSec(), (last_valid_plan_ + ros::Duration(planner_patience_)).toSec());
          as_.setAborted(msg::MoveBaseTopoResult(), error_string);
          cleanup();
          return;
        }
        ROS_WARN_THROTTLE(1.0, "Failed to make a plan in roadmap: %s, trying again.", error_string.c_str());
        //make sure to just keep looping here to try to replan
        continue;
      }

      // Beyond here we're guaranteed that makeRoadmapPlan succeeded
      
      last_valid_plan_ = ros::Time::now();
      ROS_DEBUG_STREAM_NAMED("exec", "Found roadmap plan: "
                             << gmu::toString(plan->first));
      msg::RoadmapPath msg;
      msg.path = plan->first;
      path_pub_.publish(msg);
      const unsigned best_grid = bestGrid(plan->first);
      
      
      msg::SwitchGrid srv;
      srv.request.grid = best_grid;
      if (!switch_grid_client_.call(srv))
        ROS_WARN ("Service call to switch_grid failed");
      ROS_DEBUG_NAMED("exec", "Requesting switch to grid %u", best_grid);
      unsigned i=0;
      while (ros::ok() && i++<20 && tmap::frameGrid(costmap_.getGlobalFrameID())!=best_grid)
      {
        ROS_DEBUG_NAMED("exec", "Waiting for costmap to catch up on grid switch");
        ros::Duration(0.1).sleep();
      }

      //make sure to update the frame id since it may have changed
      string global_frame = costmap_.getGlobalFrameID();
      current_waypoint = lastWaypointOnGridIndex(*plan, global_frame);
      grid_switch_ = false;

      //we need to get the pose of our target node in the grid that we're in
      gm::PoseStamped node_pose = nodePoseOnGrid(plan->first[current_waypoint],
                                                 tmap::frameGrid(global_frame));
      ROS_DEBUG_STREAM_NAMED("exec", "Aiming for waypoint " <<
                             plan->first[current_waypoint] <<
                             " at " << gmu::toString(node_pose.pose) << " on "
                             << tmap::frameGrid(global_frame));

      mb_goal.target_pose = node_pose;
      move_base_client_.sendGoal(mb_goal);
    } // End block guarded by mutex

    //now we want to monitor how close the base is to the desired goal
    ros::Rate r(progress_check_frequency_);
    bool close_enough = false;
    while(ros::ok() && !move_base_client_.getState().isDone() &&!close_enough)
    {
      {
        Lock l(mutex_);
        // if within tolerance continue to next waypoint
        if(current_waypoint < plan->first.size() - 1 &&
           (grid_switch_ || robotWithinTolerance(next_waypoint_distance_,
                                                 plan->first[current_waypoint])))
        {
          ROS_DEBUG_COND_NAMED (grid_switch_, "exec", "Switching to grid %u",
                                *current_grid_);
          close_enough = true;
          //make sure to update the last valid plan time
          last_valid_plan_ = ros::Time::now();
        }
        ROS_DEBUG_THROTTLE_NAMED (2.0, "exec", "Waiting for waypoint %u;",
                                  plan->first[current_waypoint]);
      } // End block guarded by mutex
      
      r.sleep();
    }

    //check to see if we're just stuck at the first waypoint in our plan
    bool not_going_anywhere = false;
    if ((plan->first[current_waypoint] == plan->first[0] &&
         plan->first[current_waypoint] != plan->first[plan->first.size() -1])
        && move_base_client_.getState().isDone())
    {
      not_going_anywhere = true;
    }

    // if we're not close enough to the next waypoint (by distance or
    // the grid switching)
    // or we're just not moving along the plan, sitting still at one node
    // ... then we need to check the state of move_base to see if it failed
    ROS_DEBUG_NAMED("exec", "Not going anywhere: %d, client state: %s; "
                    "Close_enough is %d", not_going_anywhere,
                    move_base_client_.getState().toString().c_str(), close_enough);
    if(!close_enough || not_going_anywhere){
      if(not_going_anywhere || move_base_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){

        //if the base aborts on its attempt to make it to a node in the plan... we need to add to our blocked edges list
        //but first we need to find what edge is actually blocked by looping backwards from our current waypoint
        bool edge_cut = false;

        unsigned int check_node = 0;
        while(check_node <= current_waypoint && check_node < plan->first.size() - 1 && !edge_cut){
          ROS_ERROR("Trying to prune");
          //we'll plan from the first waypoint in the plan to the check point
          nm::GetPlan mb_plan;

          //we need to get the pose of this node on the grid we're on
          const unsigned g = tmap::frameGrid(costmap_.getGlobalFrameID());

          const gm::PoseStamped check_node_pose =
            nodePoseOnGrid(plan->first[check_node], g);
          const gm::PoseStamped next_node_pose =
            nodePoseOnGrid(plan->first[check_node+1], g);

          mb_plan.request.start = check_node_pose;
          mb_plan.request.goal = next_node_pose;

          //if we get a empty plan back we know that we need to cut the edge
          if(!make_plan_client_.call(mb_plan) ||
             mb_plan.response.plan.poses.empty())
          {
            Lock l(mutex_);
            blocked_edges_.push_back(BlockedEdge(plan->first[check_node],
                                                 plan->first[check_node + 1],
                                                 ros::Time::now()));
            ROS_DEBUG_NAMED("exec", "Adding a blocked edge for %u->%u",
                            plan->first[check_node],
                            plan->first[check_node + 1]);
            //also, make sure to remove the edges from the roadmap
            removeBlockedEdges(blocked_edges_, roadmap_);
            edge_cut = true;
          }
          ++check_node;
        }
 
        //we'll also continue in our loop to retry planning
        if(edge_cut)
          continue;
        else if(not_going_anywhere){
           Lock l(mutex_);
          blocked_edges_.push_back(BlockedEdge(plan->first[0], plan->first[1],
                                               ros::Time::now()));
          ROS_ERROR("Adding a blocked edge for %u->%u",
                    plan->first[0], plan->first[1]);
          //also, make sure to remove the edges from the roadmap
          removeBlockedEdges(blocked_edges_, roadmap_);
           continue;
        }

        std::stringstream error_msg;
        error_msg << "MoveBase failed to make it to node" << plan->first[current_waypoint] << " state is " << move_base_client_.getState().toString();
        ROS_ERROR("%s", error_msg.str().c_str());
        as_.setAborted(msg::MoveBaseTopoResult(), error_msg.str());
        cleanup();
        return;
      }
      else
        ROS_DEBUG("Made it to node: %u", plan->first[current_waypoint]);
    }

    //if the waypoint that we've achieved is the last one... then we'll set a flag to exit the loop
    if(current_waypoint == plan->first.size() - 1)
      finished_plan = true;
  }

  //check to make sure things succeeded... just in case
  if(move_base_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    std::stringstream error_msg;
    error_msg << "MoveBase failed to make it to node" << plan->first[plan->first.size() - 1] << " state is " << move_base_client_.getState().toString();
    ROS_ERROR("%s", error_msg.str().c_str());
    as_.setAborted(msg::MoveBaseTopoResult(), error_msg.str());
    cleanup();
    return;
  }

  ROS_INFO("Made it to the goal location");
  as_.setSucceeded();
  cleanup();
}

double MoveBaseTopo::shortestPathDistance(const gm::Pose& p1)
{
  gm::PoseStamped goal;
  goal.header.frame_id = costmap_.getGlobalFrameID();
  goal.header.stamp = ros::Time::now();
  goal.pose = p1;

  vector<gm::PoseStamped> plan;

  //apparently navfn can return plans of  size 1 when it doesn't actually
  //find a valid plan... at least when that's not awesome
  if(!planner_.getPlanFromPotential(goal, plan) || plan.size() <= 1){
    return DBL_MAX;
  }

  double length = 0.0;
  for(unsigned int i = 0; i < plan.size() - 1; ++i){
    length += gmu::euclideanDistance(plan[i].pose.position, plan[i+1].pose.position);
  }
  return length;
}

void MoveBaseTopo::roadmapCB(msg::TopologicalRoadmap::ConstPtr roadmap)
{
  Lock l(mutex_);
  roadmap_ = fromRosMessage(*roadmap);
  //make sure to remove any blocked edges we have from the roadmap
  removeBlockedEdges(blocked_edges_, roadmap_);
  ROS_DEBUG_NAMED("roadmap_cb", "Received roadmap with %zu nodes",
                  roadmap->nodes.size());
}

// When we receive a localization, if it results in switching to
// a new grid, we load the new grid from the warehouse
void MoveBaseTopo::localizationCB (const gm::PoseStamped& loc)
{
  unsigned g;
  bool need_to_update;
  {
    Lock l(mutex_);
    g = tmap::frameGrid(loc.header.frame_id);
    need_to_update = (!current_grid_ || (g != *current_grid_));
  }
  if (need_to_update)
  {
    current_grid_ = g;
    nm::OccupancyGrid::ConstPtr grid = getGrid(g);
    Lock l(mutex_);
    grid_switch_ = true;
    costmap_.updateStaticMap(*grid);
  }
}


nm::OccupancyGrid::ConstPtr MoveBaseTopo::getGrid (const unsigned g) const
{
  typedef mr::MessageWithMetadata<nm::OccupancyGrid>::ConstPtr Grid;
  mr::Query query("id", g);
  ros::Rate r(10);
  while (ros::ok())
  {
    const vector<Grid> results = grids_.pullAllResults(query);
    ROS_ASSERT_MSG (results.size()<2, "Unexpected case: there were "
                    " multiple grids stored for id %u", g);
    if (results.size()==1)
      return results[0];
    ROS_WARN_THROTTLE (1.0, "Couldn't find grid %u in warehouse", g);
    r.sleep();
  }
  // Only happens on shutdown
  return Grid();
}

// When we get notified of a change in the grid collection in the
// warehouse, we check if the updated grid is the current one, and
// if so, reload it
void MoveBaseTopo::gridUpdateCB (const std_msgs::String& m)
{
  mr::Metadata metadata(m.data);
  const unsigned g = metadata.getIntField("id");
  bool need_to_update;
  {
    Lock l(mutex_);
    need_to_update = (current_grid_ && (g==*current_grid_));
  }
  if (need_to_update)
  {
    nm::OccupancyGrid::ConstPtr grid = getGrid(g);
    Lock l(mutex_);
    costmap_.updateStaticMap(*grid);
  }
}



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_topo_node");
  topological_roadmap::MoveBaseTopo mb;
  ros::spin();
  return(0);
}
