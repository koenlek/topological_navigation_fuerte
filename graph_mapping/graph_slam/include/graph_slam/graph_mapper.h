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
 * Defines the GraphMapper class, which represents a ROS node that builds a map
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_SLAM_GRAPH_MAPPER_H
#define GRAPH_SLAM_GRAPH_MAPPER_H

#include <graph_slam/localization_buffer.h>
#include <pose_graph/constraint_graph.h>
#include <pose_graph/graph_db.h>
#include <pose_graph/visualization.h>
#include <pose_graph/diff_publisher.h>
#include <graph_mapping_msgs/SaveGraph.h>
#include <graph_mapping_msgs/GetPoses.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/optional.hpp>


namespace graph_slam
{

class Odometer;

/// \brief Holds the state of the graph mapping algorithm, which runs in the background as long as this object exists.
class GraphMapper
{
public:

  /// Constructing an instance of this class starts the graph slam algorithm.  The constructor starts up timers 
  /// for graph update and visualization, service callbacks for constraints and localization, and a service callback for
  /// saving the graph, and returns.
  ///
  /// Topics and services are assumed to be in the default namespace, while parameters are looked up in the private namespace.
  GraphMapper ();

  /// Since this class can be subclassed
  virtual ~GraphMapper ();

protected:

  /// \brief Thread-safely get a copy of the current graph
  pose_graph::ConstraintGraph graphSnapshot () const;

  ros::NodeHandle param_nh_; // Needs to exist before parameters

  /****************************************
   * Parameters
   ****************************************/
  
  const double angle_threshold_;
  const double pos_threshold_;
  const std::string base_frame_, fixed_frame_, optimization_frame_;
  const ros::Duration update_duration_;
  const ros::Duration constraint_wait_;
  const std::string optimization_algorithm_;
  const ros::Duration loc_transform_wait_;
  const ros::Duration ref_transform_timeout_;
  const bool add_new_nodes_; // If this is false, we'll just localize wrt existing graph
  
  /****************************************
   * State
   ****************************************/

  boost::shared_ptr<tf::TransformListener> tf_;
  pose_graph::ConstraintGraph graph_;
  LocalizationBuffer localizations_;
  bool initialized_;
  boost::optional<tf::StampedTransform> ref_transform_; // Todo: this no longer needs to be optional
  ros::Time last_node_addition_time_;
  bool optimize_flag_; // Does the graph need to be reoptimized
  pose_graph::CachedNodeMap<geometry_msgs::Pose> ff_poses_;
  mongo_ros::MessageCollection<graph_mapping_msgs::ConstraintGraphMessage> saved_graph_;
  std::set<unsigned> last_optimized_comp_;
  pose_graph::NodePoseMap last_optimization_response_;
  

private:

  /****************************************
   * Entry points
   ****************************************/
  
  void updateGraph (const ros::TimerEvent& e);
  void visualize (const ros::TimerEvent& e) const;
  void addConstraint (graph_mapping_msgs::GraphConstraint::ConstPtr constraint);
  void saveGraphTimed (const ros::TimerEvent& e);
  void updateLocalization
  (graph_mapping_msgs::LocalizationDistribution::ConstPtr l);
  bool getPoses (graph_mapping_msgs::GetPoses::Request& req,
                 graph_mapping_msgs::GetPoses::Response& resp);
  void publishRefTransform (const ros::TimerEvent& e);

  /****************************************
   * Modifying functions.  Must hold
   * lock while calling these.
   ****************************************/
  
  void addNodeAt (const ros::Time& t);
  void saveGraph ();
  void optimize ();
  void optimizeGraphComponent (unsigned n);

  /****************************************
   * Const functions
   ****************************************/
  
  tf::Pose fixedFramePoseAt (const ros::Time& t,
                             const ros::Duration& wait) const;
  bool findNodeNear (const geometry_msgs::PoseStamped& l) const;

  /****************************************
   * Virtual functions
   ****************************************/

  /// Return a recent time when a new node can be added.  By default, returns ros::Time::now()
  /// Called while holding lock
  virtual boost::optional<ros::Time> newNodeTime ();

  /****************************************
   * Associated objects
   ****************************************/

  mutable boost::mutex mutex_; // Mutable because const operations may need to acquire a lock
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tfb_;
  pose_graph::ConstraintGraphVisualizer visualizer_;
  boost::scoped_ptr<Odometer> update_odometer_;
  ros::Subscriber constraint_sub_, loc_sub_;
  pose_graph::DiffPublisher diff_pub_;
  ros::Publisher marker_pub_, pose_pub_;
  ros::ServiceServer get_poses_srv_;
  ros::Timer update_timer_, vis_timer_, save_timer_, ref_pub_timer_;
};


} // namespace

#endif // include guard
