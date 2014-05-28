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
 * Ros node that listens to a changing map, and generates exploration commands
 *
 * \author Bhaskara Marthi
 */

#include <explore_2d/exploration.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace explore_2d
{

namespace nm=nav_msgs;
namespace mbm=move_base_msgs;
namespace al=actionlib;
namespace gm=geometry_msgs;
using std::string;

typedef actionlib::SimpleActionClient<mbm::MoveBaseAction> Client;

class ExploreNode
{
public:
  ExploreNode();
  void spin();

private:

  void mapCallback (const nm::OccupancyGrid& map);

  ros::NodeHandle nh_;
  boost::mutex mutex_;

  const string global_frame_;

  bool have_received_map_;
  bool initialized_;
  Explorer explorer_;
  
  Client move_base_client_;
  ros::Subscriber map_sub_;
};


ExploreNode::ExploreNode() :
  global_frame_("/map"),
  have_received_map_(false), initialized_(false), explorer_(0.5),
  move_base_client_("move_base", true), 
  map_sub_(nh_.subscribe("map", 10, &ExploreNode::mapCallback, this))
{
  ros::Duration d(5.0);
  while (ros::ok()) {
    if (move_base_client_.waitForServer(d)) {
      ROS_INFO ("Connected to move_base server");
      break;
    }
    ROS_INFO ("Waiting for connection to move_base server");
  }
  initialized_ = true;
}


void ExploreNode::mapCallback (const nm::OccupancyGrid& map)
{
  ros::Rate r(1.0);
  if (initialized_) {
    boost::mutex::scoped_lock l(mutex_);
    explorer_.updateOccupancyGrid(map);
    have_received_map_ = true;
  }
}


void ExploreNode::spin ()
{
  ros::Rate r(1.0);
  while (ros::ok()) {
    ros::spinOnce();

    gm::Pose goal_pose;
    {
      boost::mutex::scoped_lock l(mutex_);
      if (!have_received_map_) {
        ROS_INFO_THROTTLE(5.0, "Waiting for a map before sending goals");
        continue;
      }
      goal_pose = explorer_.nextNavGoal();
    }
    
    mbm::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = global_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;
    move_base_client_.sendGoal(goal);
    ROS_INFO_STREAM_NAMED ("goals", "Sending goal " << goal_pose);
    while (ros::ok()) {
      al::SimpleClientGoalState state = move_base_client_.getState();
      if (state == al::SimpleClientGoalState::PENDING ||
          state == al::SimpleClientGoalState::ACTIVE) {
        r.sleep();
        continue;
      }
      else if (state == al::SimpleClientGoalState::SUCCEEDED ||
               state == al::SimpleClientGoalState::ABORTED) {
        ROS_INFO_STREAM_NAMED ("goals", "Goal finished with result " << state.toString());
        break;
        
      }
      else {
        ROS_ERROR_STREAM ("Unexpected client state " << state.toString() << "; exiting");
        return;
      }
    }
  }
}




} // namespace



int main (int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  explore_2d::ExploreNode n;
  n.spin();
  return 0;
}
