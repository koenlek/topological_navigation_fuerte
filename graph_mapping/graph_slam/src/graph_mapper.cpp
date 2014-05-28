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
 * Implementation of graph_mapper.h
 *
 * \author Bhaskara Marthi
 */

#include <graph_slam/graph_mapper.h>
#include "odometer.h"
#include <graph_slam/exception.h>
#include <graph_mapping_utils/utils.h>
#include <pose_graph/spa_2d_conversion.h>
#include <pose_graph/message_conversion.h>
#include <pose_graph/graph_search.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

namespace graph_slam
{

namespace pg=pose_graph;
namespace msg=graph_mapping_msgs;
namespace util=graph_mapping_utils;
namespace gm=geometry_msgs;
namespace vm=visualization_msgs;
namespace mr=mongo_ros;
using std::string;
using std::vector;
typedef msg::LocalizationDistribution::ConstPtr LocPtr;
typedef boost::optional<ros::Time> MaybeTime;


struct Lock
{
  
  Lock(boost::mutex& m)
  {
    ROS_DEBUG_STREAM_NAMED ("lock", "Waiting for lock");
    l = boost::mutex::scoped_lock(m);
    ROS_DEBUG_STREAM_NAMED ("lock", "Acquired lock");
  }

  ~Lock()
  {
    ROS_DEBUG_STREAM_NAMED ("lock", "Releasing lock");
  }

  boost::mutex::scoped_lock l;
};


GraphMapper::GraphMapper () :
  param_nh_("~"), 
  angle_threshold_(util::getParam<double>(param_nh_, "angle_displacement_threshold")),
  pos_threshold_(util::getParam<double>(param_nh_, "position_displacement_threshold")),
  base_frame_(util::getParam<string>(param_nh_, "base_frame")),
  fixed_frame_(util::getParam<string>(param_nh_, "fixed_frame")),
  optimization_frame_(util::getParam<string>(param_nh_, "optimization_frame", "graph_optimization")),
  update_duration_(util::duration(util::getParam<double>(param_nh_, "update_rate", 2.0))),
  constraint_wait_(ros::Duration(util::getParam<double>(param_nh_, "constraint_wait_time", 0.5))),
  optimization_algorithm_(util::getParam<string>(param_nh_, "optimization_algorithm", "spa")),
  loc_transform_wait_(0.1), ref_transform_timeout_(8.0),
  add_new_nodes_(util::getParam<bool>(param_nh_, "add_new_nodes", true)),
  tf_(new tf::TransformListener()), 
  localizations_(util::getParam<int>(param_nh_, "localization_buffer_size", 20), fixed_frame_, base_frame_, tf_, 5.0),
  initialized_(false), optimize_flag_(true), ff_poses_("graph_mapping", "fixed_frame_poses"),
  saved_graph_(mr::MessageCollection<msg::ConstraintGraphMessage> ("graph_mapping", "constraint_graph")),
  visualizer_(util::getParam<bool>(param_nh_, "visualize_node_ids", false)),
  update_odometer_(new Odometer(tf_.get(), base_frame_, fixed_frame_, ros::Duration(15.0))),
  constraint_sub_(nh_.subscribe("graph_constraints", 100, &GraphMapper::addConstraint, this)),
  loc_sub_(nh_.subscribe("graph_localization", 10, &GraphMapper::updateLocalization, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 5)),
  pose_pub_(nh_.advertise<gm::PoseStamped>("optimized_pose", 5)),
  get_poses_srv_(nh_.advertiseService("get_node_poses", &GraphMapper::getPoses,
                                      this)),
  update_timer_(nh_.createTimer(update_duration_, &GraphMapper::updateGraph, this)),
  vis_timer_(nh_.createTimer(util::duration(util::getParam<double>(param_nh_, "visualization_rate", 1.0)),
                             &GraphMapper::visualize, this)),
  save_timer_(nh_.createTimer(util::duration(util::getParam<double>(param_nh_, "save_rate", 0.1)),
                              &GraphMapper::saveGraphTimed, this)),
  ref_pub_timer_(nh_.createTimer(util::duration(util::getParam<double>(param_nh_, "ref_transform_pub_rate_", 5.0)),
                                 &GraphMapper::publishRefTransform, this))
  
{
  Lock l(mutex_);

  /// Assuming at least one saved graph exists, get the most recent one
  mr::QueryResults<msg::ConstraintGraphMessage>::range_t graphs =
    saved_graph_.queryResults(mr::Query(), false, "creation_time", false);
  if (graphs.first != graphs.second)
    graph_ = pg::constraintGraphFromMessage(**graphs.first);

  diff_pub_.setGraph(graph_);
  ref_transform_ = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), fixed_frame_, optimization_frame_);

  // Initialize ref transform to be not quite the same as the fixed frame; this makes debugging easier
  ref_transform_->getOrigin().setX(0.42);
  ref_transform_->getOrigin().setY(-0.42);

  ROS_ASSERT_MSG (add_new_nodes_ || !graph_.allNodes().empty(),
                  "Can't run in localization-only mode without a saved graph.");

  initialized_ = true;
}

GraphMapper::~GraphMapper ()
{
}

/************************************************************
 * Visualization
 ***********************************************************/

void GraphMapper::visualize (const ros::TimerEvent& e) const
{
  if (!initialized_)
    return;
  visualizer_.visualize(graphSnapshot());

  // Also visualize the localization
  boost::optional<gm::PoseStamped> l = localizations_.lastLocalization();
  if (l) {
    const unsigned ref = util::refNode(*l);
    if (graph_.hasOptimizedPose(ref)) {
      const tf::Pose p = graph_.getOptimizedPose(ref);
      vm::Marker m;
      m.ns = "graph_localization";
      m.header.frame_id = optimization_frame_;
      m.header.stamp = ros::Time::now();
      m.id = 1;
      m.type = vm::Marker::ARROW;
      m.action = vm::Marker::ADD;
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.points.resize(2);
      m.points[0] = util::toPose(p).position;
      gm::PoseStamped opt_pose;
      opt_pose.pose = util::transformPose(p, l->pose);
      opt_pose.header.frame_id = optimization_frame_;
      opt_pose.header.stamp = ros::Time::now();
      pose_pub_.publish(opt_pose);
      m.points[1] = opt_pose.pose.position;
      m.color.r = 0.6;
      m.color.g = 0.2;
      m.color.b = 0.8;
      m.color.a = 1.0;
      marker_pub_.publish(m);
    }
    else
      ROS_DEBUG_STREAM_NAMED ("visualization", "Not visualizing localization: "
                              << ref << " has no optimized pose");
  }
  else
    ROS_INFO_THROTTLE (3, "Not visualizing localization because "
                       "no localizations received yet");
}

/************************************************************
 * Localization
 ***********************************************************/

/// Use the localization to recompute the transform between the
/// fixed and optimization frames, and publish it.
void GraphMapper::updateLocalization (LocPtr m)
{
  ROS_DEBUG_STREAM_NAMED ("localization", "Updating localization");
  ROS_ASSERT (m->samples.size() == 1);
  const gm::PoseStamped& p = m->samples[0];
  try {
    const tf::Pose fixed_frame_pose =
      fixedFramePoseAt(m->stamp, loc_transform_wait_);
    const unsigned ref = util::refNode(p);
    Lock lock(mutex_);
    if (graph_.hasOptimizedPose(ref)) {
      const tf::Pose ref_pose = graph_.getOptimizedPose(ref);
      const tf::Pose opt_pose = util::absolutePose(ref_pose, p.pose);
      ref_transform_ =
        tf::StampedTransform(util::transformBetween(opt_pose, fixed_frame_pose),
                             m->stamp, fixed_frame_, optimization_frame_);
    }
    else {
      ROS_DEBUG_STREAM_NAMED ("localization", "Not updating ref transform due "
                              "to lack of optimized pose for " << ref);
      optimize_flag_ = true;
    }
  }
  catch (tf::TransformException& e) {
    ROS_WARN_STREAM ("Transform exception during localization update: "
                     << e.what() << "; skipping localization update");
  }

}

void GraphMapper::publishRefTransform (const ros::TimerEvent& e)
{
  Lock l(mutex_);

  // Publish the reference transform if possible
  if (ref_transform_) {
    const ros::Time t = ros::Time::now();
    const ros::Duration age = t - ref_transform_->stamp_;
    if (age > ref_transform_timeout_) 
      ROS_WARN_STREAM_THROTTLE_NAMED (3.0, "localization",
                                      "Ref transform is " << age <<
                                      " seconds old.");
    tf::StampedTransform tr = *ref_transform_;
    tr.stamp_ = t;
    tfb_.sendTransform(tr);
    ROS_DEBUG_STREAM_NAMED ("localization", "Published ref transform");
  }
}


/************************************************************
 * Edge addition
 ***********************************************************/

void GraphMapper::addConstraint (msg::GraphConstraint::ConstPtr constraint)
{
  Lock l(mutex_);

  const unsigned e = graph_.addEdge (constraint->src,
                                     constraint->dest,
                                     constraint->constraint);
  optimize_flag_ = true;
  ROS_DEBUG_STREAM_NAMED ("update_graph", "Added edge from " <<
                          constraint->src << " to " << constraint->dest);

  msg::ConstraintGraphDiff diff;
  diff.new_edges.resize(1);
  diff.new_edges[0].id = e;
  diff.new_edges[0].constraint = *constraint;
  diff_pub_.addDiff(&diff);
  
}

/************************************************************
 * Graph node addition
 ***********************************************************/

tf::Pose GraphMapper::fixedFramePoseAt (const ros::Time& t,
                                        const ros::Duration& wait) const
{
  tf::StampedTransform tr;
  tf_->waitForTransform(fixed_frame_, base_frame_, t, wait);
  tf_->lookupTransform(fixed_frame_, base_frame_, t, tr);
  return tr; // Slicing, which is ok
}


void GraphMapper::addNodeAt (const ros::Time& t)
{
  const unsigned n = graph_.addNode();
  optimize_flag_ = true;
  last_node_addition_time_ = t;

  const tf::Pose ff_pose = fixedFramePoseAt(t, update_duration_ * 0.5);
  ff_poses_.set(n, util::toPose(ff_pose));

  msg::ConstraintGraphDiff diff;
  diff.new_node_timestamps.push_back(t);
  diff.new_nodes.resize(1);
  diff.new_nodes[0].id = n;
  diff_pub_.addDiff(&diff);

  ROS_DEBUG_STREAM_NAMED ("update_graph", "Added node " << n << " at " << t);
}

/// Is there a node that's near l in the graph as well as metrically?
bool GraphMapper::findNodeNear (const gm::PoseStamped& p) const
{
  const unsigned ref = util::refNode(p);
  const tf::Pose ref_pose = graph_.getOptimizedPose(ref);
  const tf::Pose opt_pose = util::absolutePose(ref_pose, p.pose);
  pg::OptimizedDistancePredicate pred(graph_, opt_pose.getOrigin(),
                                      pos_threshold_*4);
  pg::NodeSet nodes = filterNearbyNodes(graph_, ref, pred);
  BOOST_FOREACH (const unsigned n, nodes) {
    const tf::Pose p = util::relativePose(graph_.getOptimizedPose(n), opt_pose);
    const double angle_displacement = fabs(tf::getYaw(p.getRotation()));
    const double pos_displacement = p.getOrigin().length();
    if (angle_displacement < angle_threshold_ &&
        pos_displacement < pos_threshold_) {
      return true;
    }
  }
  return false;
}

void GraphMapper::updateGraph (const ros::TimerEvent& e)
{
  bool add_new_node = false;
  if (!initialized_)
    return;

  // Figure out whether to add node, while holding lock
  {
    Lock l(mutex_);

    add_new_node = (graph_.allNodes().size() == 0);
    if (!add_new_node) {
      const tf::Pose disp = update_odometer_->getDisplacement();
      if (disp.getOrigin().length() > pos_threshold_ ||
          fabs(tf::getYaw(disp.getRotation())) > angle_threshold_) {
        boost::optional<gm::PoseStamped> last_loc =
          localizations_.lastLocalization();
        add_new_node = true;
        /*
        if (last_loc) {
          try {
            if (!findNodeNear(*last_loc))
              add_new_node = true;
          }
          catch (pg::NoOptimizedPoseException& e) {
            ROS_DEBUG_STREAM_NAMED ("update_graph", "Not adding node due to "
                                    "a NoOptimizedPoseException");
          }
        }
        */
      }
    }
  }

    
  if (add_new_node && add_new_nodes_) {
    // This is called without lock, in case it calls a service or something
    MaybeTime t = newNodeTime();
    if (t) {
      Lock l(mutex_);
      if (graph_.allNodes().empty() || localizations_.hasLocalization(*t)) {
        addNodeAt(*t);
        update_odometer_->reset();
      }
      else {
        ROS_DEBUG_STREAM_NAMED ("update_graph", "Not adding node at time " << *t
                                << " due to lack of localization");
      }
    }
  }

  /// Also, optimize if necessary
  optimize();
}


MaybeTime GraphMapper::newNodeTime ()
{
  return MaybeTime(ros::Time::now());
}


/************************************************************
 * Optimization
 ***********************************************************/

void GraphMapper::optimize ()
{
  if (optimize_flag_) {
    boost::optional<gm::PoseStamped> last_loc =
      localizations_.lastLocalization();
    if (last_loc) 
      optimizeGraphComponent(util::refNode(*last_loc));
    else 
      ROS_DEBUG_STREAM_NAMED ("optimize", "Not optimizing since "
                              "we don't have a last localization");
  }
}

/// \todo This calls optimizer while holding lock
void GraphMapper::optimizeGraphComponent (const unsigned n)
{
  pg::ConstraintGraph copy;
  pg::NodeSet comp;
  pg::NodePoseMap init;
  {
    Lock l(mutex_);
    copy = graph_;
    comp = componentContaining(graph_, n);
    BOOST_FOREACH (unsigned n, comp)
      init[n] = util::toPose(*ff_poses_.get(n));
  }

  pg::NodePoseMap opt_poses;
  ROS_DEBUG_STREAM_NAMED ("optimize", "Optimizing component with "
                          << comp.size() << " nodes");
  if (optimization_algorithm_ == "spa")
    opt_poses = pg::optimizeGraph2D(copy, init, comp);
  else {
    ROS_ASSERT_MSG (optimization_algorithm_ == "none", "Unknown optimization "
                    "algorithm %s", optimization_algorithm_.c_str());
    opt_poses = init;
  }

  Lock l(mutex_);
  graph_.setOptimizedPoses(opt_poses);
  last_optimized_comp_ = comp;
  last_optimization_response_ = opt_poses;
  saveGraph();
  ROS_DEBUG_STREAM_NAMED ("optimize", "Optimized poses of " <<
                          opt_poses.size() << " nodes");
  optimize_flag_ = false;
}


bool GraphMapper::getPoses (msg::GetPoses::Request& req, msg::GetPoses::Response& resp)
{
  pg::NodeSet comp;
  pg::ConstraintGraph graph_copy;
  pg::NodePoseMap init_estimates;
  pg::NodePoseMap opt_poses;
  bool optimize=false;

  {
    // While holding lock, figure out if need to optimize, copy info
    Lock l(mutex_);
    if (req.nodes.empty())
    {
      resp.error_code = msg::GetPoses::Response::EMPTY_NODE_SET;
      return true;
    }

    // Check that all nodes are in a single connected component
    // We'll optimize this entire component
    const unsigned n(req.nodes[0]);
    comp = componentContaining(graph_, n);
    BOOST_FOREACH (const unsigned id, req.nodes)
    {
      if (!util::contains(comp, id))
      {
        resp.error_code = msg::GetPoses::Response::DISCONNECTED_NODES;
        return true;
      }
    }

    // Is this component the cached one?  If so, reuse the solution.
    // \todo This ignores if new edges have been added
    if (comp == last_optimized_comp_)
    {
      opt_poses = last_optimization_response_;
    }
    else
    {
      BOOST_FOREACH (unsigned n, comp)
        init_estimates[n] = util::toPose(*ff_poses_.get(n));
      optimize = true;
      graph_copy = graph_;
    }
  }

  // Optimize, without lock
  if (optimize)
  {
    ROS_DEBUG_STREAM_NAMED ("optimize", "Optimizing component with "
                            << comp.size() << " nodes");
    opt_poses = pg::optimizeGraph2D(graph_copy, init_estimates, comp);
  }

  // Write information back, with lock.
  // It's fine that the lock was dropped in the middle, since nobody else
  // modifies these variables.
  {
    Lock l(mutex_);
    if (optimize)
    {
      last_optimized_comp_ = comp;
      last_optimization_response_ = opt_poses;
    }
  }

  // Fill in response
  resp.poses.reserve(req.nodes.size());
  BOOST_FOREACH (const pg::NodePoseMap::value_type& e, opt_poses) 
    resp.poses.push_back(util::toPose(e.second));
  resp.error_code = msg::GetPoses::Response::SUCCESS;
  return true;
}


/************************************************************
 * Save/copy graph
 ***********************************************************/

// For now, we save only after optimization
void GraphMapper::saveGraphTimed (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  //saveGraph(); 
}

void GraphMapper::saveGraph ()
{
  ROS_DEBUG_STREAM_NAMED ("save", "Saving graph with " <<
                          graph_.allNodes().size() << " nodes");
  saved_graph_.insert(pg::constraintGraphToMessage(graph_));
}

pg::ConstraintGraph GraphMapper::graphSnapshot () const
{
  Lock l(mutex_);
  return graph_;
}




} // namespace
