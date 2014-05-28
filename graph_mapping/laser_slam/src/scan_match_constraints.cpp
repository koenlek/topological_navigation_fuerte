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

#include <laser_slam/scan_match_constraints.h>
#include <pose_graph/graph_search.h>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
#include <graph_mapping_utils/utils.h>
#include <visualization_msgs/Marker.h>
#include <pose_graph/exception.h>
#include <geometry_msgs/PoseStamped.h>

namespace laser_slam
{

namespace pg=pose_graph;
namespace util=graph_mapping_utils;
namespace msg=graph_mapping_msgs;
namespace ksm=karto_scan_matcher;
namespace gm=geometry_msgs;
namespace vm=visualization_msgs;
namespace gs=graph_slam;

using std::string;
using util::toString;

typedef boost::mutex::scoped_lock Lock;
typedef std::map<unsigned, unsigned> NodeMap;
typedef boost::optional<unsigned> MaybeNode;

const unsigned LOCALIZATION_BUFFER_SIZE=50;
const double LOOP_MATCH_FINE_MULTIPLIER=0.2;
const double POLL_INC=0.1;
const double POLL_MAX=1;
const double MAX_EXTRAPOLATION=5.0;
const double EXPECTED_NODE_DISTANCE=0.3;
const double DISTANCE_UPDATE_INC=0.5;
const double MAX_SPEED=1.0;

ScanMatchConstraints::ScanMatchConstraints (ros::NodeHandle p_nh, TfPtr tf) :
  fixed_frame_(util::getParam<string>(p_nh, "fixed_frame")),
  base_frame_(util::getParam<string>(p_nh, "base_frame")),
  laser_frame_(util::getParam<string>(p_nh, "laser_frame")),
  opt_frame_(util::searchParam<string>(p_nh, "optimization_frame")),
  local_window_size_(util::getParam<double>(p_nh, "local_window_size", 10.0)),
  running_scan_match_size_(util::getParam<double>(p_nh, "running_scan_match_size", 0.3)),
  running_scan_match_res_(util::getParam<double>(p_nh, "running_scan_match_resolution", 0.01)),
  near_link_min_chain_size_(util::getParam<int>(p_nh, "near_link_min_chain_size", 8)),
  chain_distance_threshold_(util::getParam<double>(p_nh, "chain_distance_threshold", 2.0)),
  loop_scan_match_size_(util::getParam<double>(p_nh, "loop_scan_match_size", 5.0)),
  loop_scan_match_res_(util::getParam<double>(p_nh, "loop_scan_match_resolution", 0.05)),
  loop_match_window_size_(util::getParam<double>(p_nh, "loop_closure_window_size", 5.0)),
  loop_match_min_chain_size_(util::getParam<int>(p_nh, "loop_closure_min_chain_size", 10)),
  min_loop_response_(util::getParam<double>(p_nh, "min_loop_closure_response", 0.7)),
  max_loop_match_variance_(util::getParam<double>(p_nh, "max_loop_match_variance", 0.16)), 
  min_sequential_response_(util::getParam<double>(p_nh, "min_sequential_response", 0.7)),
  max_nbd_size_(util::getParam<double>(p_nh, "scan_match_nbd_size", 20)),
  running_buffer_size_(util::getParam<int>(p_nh, "running_buffer_size", 50)),
  initialized_(false), distance_since_last_added_node_(0), running_nodes_(running_buffer_size_),
  tf_(tf), before_("graph_mapping", PREV_NODE_TOPIC), after_("graph_mapping", NEXT_NODE_TOPIC),
  scans_("graph_mapping", "scans"), ff_poses_("graph_mapping", "fixed_frame_poses"),
  loc_buf_(LOCALIZATION_BUFFER_SIZE, fixed_frame_, base_frame_, tf, MAX_EXTRAPOLATION),
  diff_sub_(boost::bind(&ScanMatchConstraints::addConstraints, this, _1, _2)),
  loc_sub_(nh_.subscribe("graph_localization", 5, &ScanMatchConstraints::locCallback, this)),
  constraint_pub_(nh_.advertise<msg::GraphConstraint>("graph_constraints", 10)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 1)),
  pose_pub_(nh_.advertise<gm::PoseStamped>("opt_pose_estimate", 1)),
  get_poses_client_(nh_.serviceClient<msg::GetPoses>("get_node_poses")),
  distance_update_timer_(nh_.createTimer(ros::Duration(DISTANCE_UPDATE_INC),
                                         &ScanMatchConstraints::updateDistanceTravelled, this))
{
  Lock l(mutex_);

  // Look up and store the transform between the base and laser frames, which is assumed fixed
  while (ros::ok()) {
    ros::Time t = ros::Time::now();
    if (tf_->waitForTransform(base_frame_, laser_frame_, t, ros::Duration(5.0))) {
      gm::PoseStamped id, laser_pose;
      id.pose.orientation.w = 1.0;
      id.header.stamp = t;
      id.header.frame_id = laser_frame_;
      tf_->transformPose(base_frame_, id, laser_pose);
      laser_offset_ = util::toPose(laser_pose.pose);
      break;
    }
    ROS_INFO_STREAM ("Waiting for transform between " << base_frame_ << " and " << laser_frame_);
  }

  initialized_ = ros::ok();
  ROS_DEBUG_NAMED ("init", "Initialized scan match constraint generator");
}

void ScanMatchConstraints::locCallback (msg::LocalizationDistribution::ConstPtr m)
{
  Lock lock(mutex_);
  ROS_ASSERT (m->samples.size()==1);
  const gm::PoseStamped& l = m->samples[0];
  const unsigned ref = util::refNode(l);
  if (!(running_nodes_.size() > 0 && *(--running_nodes_.end()) == ref)) 
    running_nodes_.push_back(ref);
  const tf::Vector3 p = fixedFramePose(ref).getOrigin();
  const double max_dist = running_buffer_size_ * EXPECTED_NODE_DISTANCE;
  while (running_nodes_.size() > 3 && p.distance(fixedFramePose(running_nodes_[0]).getOrigin()) > max_dist)
    running_nodes_.pop_front();
  
}



void ScanMatchConstraints::addConstraints
(OptionalDiff diff, const pose_graph::ConstraintGraph& orig)
{

  pg::ConstraintGraph g = orig; // Copy so we can set poses
  ROS_ASSERT(0 < LOOP_MATCH_FINE_MULTIPLIER && LOOP_MATCH_FINE_MULTIPLIER < 1);
  if (diff && diff->new_nodes.size() == 1 && g.allNodes().size() > 1) {    

    // Wait until scan for new node is in db
    const unsigned n(diff->new_nodes[0].id);

    ros::Time t;
    while (ros::ok()) {
      try 
      {
        t = scans_.get(n)->scan.header.stamp;
        break;
      }
      catch (pg::DataNotFoundException& e) 
      {
        ros::Duration(POLL_INC).sleep();
      }
    }

    // Wait until we have a localization for the time scan was taken
    unsigned j=0;
    gm::PoseStamped graph_loc;
    while (ros::ok()) {
      if (loc_buf_.hasLocalization(t)) {
        graph_loc = loc_buf_.localizationAt(t); // Mild race condition
        break;
      }
      if (++j>5) {
        ROS_WARN_STREAM_NAMED ("scan_match_constraints", "Not generating constraints to " << n <<
                               " because localization for time " << t << " unavailable.");
        return;        
      }
      ros::Duration(POLL_INC).sleep();
    }

    ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Generating scan match constraints given new node "
                            << n << " and ref node " << util::refNode(graph_loc));
    Lock l(mutex_);

    // Initialize matchers if necessary
    if (!sequential_matcher_) {
      sequential_matcher_.reset(new ksm::KartoScanMatcher(scans_.get(n)->scan, util::projectToPose2D(laser_offset_),
                                                          running_scan_match_size_,
                                                          running_scan_match_res_));
      ksm::DoubleVector resolutions, sizes;
      sizes.push_back(loop_scan_match_size_);
      resolutions.push_back(loop_scan_match_res_);

      // Add a fine-grained match after the initial one
      sizes.push_back(loop_scan_match_size_*LOOP_MATCH_FINE_MULTIPLIER);
      resolutions.push_back(loop_scan_match_res_*LOOP_MATCH_FINE_MULTIPLIER);
      loop_matcher_.reset(new ksm::KartoScanMatcher(scans_.get(n)->scan, util::projectToPose2D(laser_offset_),
                                                    sizes, resolutions));

      ROS_DEBUG_NAMED ("scan_match_constraints", "Matchers initialized");
    }
    ROS_ASSERT(loop_matcher_);

    {
      const unsigned ref = util::refNode(graph_loc);
      pg::NodeSet comp = pg::componentContaining(g, ref);

      optimizeGraph(&g, &optimized_poses_, comp, get_poses_client_);
      ROS_ASSERT (!util::contains(optimized_poses_, n));
    }

    pg::NodeSet running_nodes = getRunningNodes(g, n, graph_loc);
    ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Running nodes are "
                            << util::toString(running_nodes));
    ConstraintVec constraints = getRunningConstraints(n, g, graph_loc,
                                                      running_nodes);
    /*
    const ConstraintVec near_link_constraints = getNearLinkConstraints(n, g, graph_loc, running_nodes);
    constraints.insert(constraints.end(), near_link_constraints.begin(), near_link_constraints.end());
    */
    const ConstraintVec loop_constraints = getLoopConstraints(n, g, graph_loc, running_nodes);
    constraints.insert(constraints.end(), loop_constraints.begin(), loop_constraints.end());

    
    BOOST_FOREACH (const msg::GraphConstraint& c, constraints)
      constraint_pub_.publish(c);

    if (last_added_node_ && distance_since_last_added_node_ < chain_distance_threshold_)
      addSequenceLink(*last_added_node_, n);
    
    last_added_node_ = n;
    distance_since_last_added_node_ = 0.0;
    ROS_DEBUG_NAMED ("scan_match_constraints", "Added %zu constraints",
                     constraints.size());
  }
}

pg::NodeSet ScanMatchConstraints::getRunningNodes (const pg::ConstraintGraph& g, const unsigned new_node,
                                                   const gm::PoseStamped& graph_loc) const
{
  
  LocalizedScan::ConstPtr scan = scans_.get(new_node);
  const unsigned ref = util::refNode(graph_loc);
  const tf::Pose opt_pose_estimate = g.getOptimizedPose(ref)*util::toPose(graph_loc.pose);

  const gm::Point barycenter = util::transformPoint(opt_pose_estimate, scan->barycenter);
  BarycenterDistancePredicate pred(g, scans_, barycenter, local_window_size_);
  const pg::NodeSet nearby = pg::filterNearbyNodes(g, ref, pred);
  
  Chain chain = getChain(ref, nearby, pg::NodeSet());
  pg::NodeSet running(chain.begin(), chain.end());
  return running;
}

tf::Pose ScanMatchConstraints::fixedFramePose (const unsigned n) const
{
  return util::toPose(*ff_poses_.get(n));
}

void ScanMatchConstraints::updateDistanceTravelled (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  const ros::Time t = ros::Time::now();
  const ros::Duration inc(DISTANCE_UPDATE_INC);
  const ros::Time prev = t - inc;
  util::MaybeTransform tr = util::getTransform(*tf_, base_frame_, prev,
                                               base_frame_, t, fixed_frame_,
                                               inc);
  if (tr) 
    distance_since_last_added_node_ += tr->getOrigin().length();
  else
  {
    ROS_WARN_STREAM ("Couldn't get transform between times " << t << " and " << prev <<
                     " in distance update.");
    distance_since_last_added_node_ += DISTANCE_UPDATE_INC * MAX_SPEED;
  }
}

ConstraintVec ScanMatchConstraints::getNearLinkConstraints (const unsigned new_node, const pg::ConstraintGraph& g,
                                                            const gm::PoseStamped& graph_loc,
                                                            const pg::NodeSet& running_nodes) const
{
  ConstraintVec constraints;
  LocalizedScan::ConstPtr scan = scans_.get(new_node);
  const unsigned ref = util::refNode(graph_loc);
  const tf::Pose opt_pose_estimate = g.getOptimizedPose(ref)*util::toPose(graph_loc.pose);

  const gm::Point barycenter = util::transformPoint(opt_pose_estimate, scan->barycenter);
  BarycenterDistancePredicate pred(g, scans_, barycenter, local_window_size_);
  const pg::NodeSet nearby = pg::filterNearbyNodes(g, ref, pred);

  pg::NodeSet processed(running_nodes.begin(), running_nodes.end());

  BOOST_FOREACH (const unsigned n, nearby) {
    const Chain chain = getChain(n, nearby, processed);
    if (chain.size() >= near_link_min_chain_size_) {
      const pg::NodeSet nodes(chain.begin(), chain.end());
      const pg::NodeSet node_subset = util::sampleSubset(nodes, max_nbd_size_);
      const ksm::ScanMatchResult res =
        scanMatchNodes(g, sequential_matcher_, node_subset, scans_,
                       scan->scan, opt_pose_estimate, laser_offset_);
      if (res.response > min_sequential_response_) {
        const msg::GraphConstraint constraint = makeConstraint(g, new_node, res, nodes, scan);
        constraints.push_back(constraint);
        ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Adding near chain constraint to " << constraint.src);
      }
      processed.insert(chain.begin(), chain.end());
    }
  }
  return constraints;
}

ConstraintVec ScanMatchConstraints::getLoopConstraints (const unsigned n, const pg::ConstraintGraph& g,
                                                        const gm::PoseStamped& graph_loc,
                                                        const pg::NodeSet& running) const
{
  ConstraintVec constraints;
  LocalizedScan::ConstPtr scan = scans_.get(n);
  const unsigned ref = util::refNode(graph_loc);
  const tf::Pose opt_pose_estimate = g.getOptimizedPose(ref)*util::toPose(graph_loc.pose);

  const gm::Point barycenter = util::transformPoint(opt_pose_estimate, scan->barycenter);
  BarycenterDistancePredicate pred(g, scans_, barycenter, loop_match_window_size_);
  typedef boost::filtered_graph<pg::Graph, pg::AllEdges, BarycenterDistancePredicate> FilteredGraph;
  const FilteredGraph filtered(g.graph(), pg::AllEdges(), pred);

  // 3. Figure out the connected components of the filtered graph

  // a. Set up color map
  typedef std::map<pg::GraphVertex, boost::default_color_type> ColorMap;
  ColorMap cmap;
  boost::associative_property_map<ColorMap> color_pmap(cmap);
  
  // b. Output map from vertex to component
  typedef std::map<pg::GraphVertex, unsigned> ComponentMap;
  ComponentMap comp_map;
  boost::associative_property_map<ComponentMap> comp_pmap(comp_map);

  // c. Connected components
  const unsigned num_comps = connected_components(filtered, comp_pmap, color_map(color_pmap));
  std::vector<pg::NodeSet> comps(num_comps);
  BOOST_FOREACH (const ComponentMap::value_type& e, comp_map) {
    comps[e.second].insert(filtered[e.first].id);
  }

  // d. We don't try loop matches against components containing the current
  // node or running nodes
  std::set<unsigned> not_loop_matchable;
  {
    ComponentMap::const_iterator pos = comp_map.find(g.idVertex(n));
    if (pos!=comp_map.end())
      not_loop_matchable.insert(pos->second);
    BOOST_FOREACH (const unsigned running_node, running) {
      pos = comp_map.find(g.idVertex(running_node));
      if (pos!=comp_map.end())
        not_loop_matchable.insert(pos->second);
    }
  }
  
  ROS_DEBUG_STREAM_COND_NAMED
    (num_comps>1, "loop_closure", "There were " << num_comps <<
     " components, with nonmatchable components " <<
     util::toString(not_loop_matchable));

  // 4. Do a loop match against each other component and publish successful
  // ones as constraints
  for (unsigned comp=0; comp<num_comps; comp++)
  {
    if (util::contains(not_loop_matchable, comp))
      continue;
    if (comps[comp].size() < (unsigned)loop_match_min_chain_size_)
    {
      ROS_DEBUG_STREAM_NAMED ("loop_closure", "Skipping component of size "
                              << util::toString(comps[comp]));
      continue;
    }
    const pg::NodeSet node_subset =
      util::sampleSubset(comps[comp], max_nbd_size_);
    ksm::ScanMatchResult res =
      scanMatchNodes(g, loop_matcher_, node_subset, scans_, scan->scan,
                     opt_pose_estimate, laser_offset_);
    ROS_DEBUG_STREAM_NAMED ("loop_closure", "Potential loop match " << comp
                            << " of size " << loop_match_min_chain_size_ <<
                            " has response " << res.response <<
                            " and covariances " << res.cov(0,0) << ", " <<
                            res.cov(1,1));
    if (res.response > min_loop_response_ &&
        res.cov(0,0) < max_loop_match_variance_ &&
        res.cov(1,1) < max_loop_match_variance_) {
      ROS_DEBUG_STREAM_NAMED ("loop_closure",
                              "Adding loop constraint to component " << comp);
      constraints.push_back(makeConstraint(g, n, res, comps[comp], scan));
    }
  }
  return constraints;
}

ConstraintVec ScanMatchConstraints::getRunningConstraints
(const unsigned n, const pg::ConstraintGraph& g,
 const gm::PoseStamped& graph_loc, const pg::NodeSet& running) const
{
  ConstraintVec constraints;
  const unsigned ref = util::refNode(graph_loc);
  LocalizedScan::ConstPtr scan = scans_.get(n);
  
  // Initialize using localization estimate at the time the scan was taken
  const tf::Pose opt_pose_estimate =
    g.getOptimizedPose(ref)*util::toPose(graph_loc.pose);

  // Scan match
  const pg::NodeSet nodes = util::sampleSubset(running, max_nbd_size_);
  ksm::ScanMatchResult res =
    scanMatchNodes(g, sequential_matcher_, nodes, scans_, scan->scan,
                   opt_pose_estimate, laser_offset_);

  
  const msg::GraphConstraint running_constraint = makeConstraint(g, n, res,
                                                                 running, scan);
  constraints.push_back(running_constraint);
  ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Added running constraint "
                          "to " << running_constraint.src);
  
  // Always add constraint to last added node if it's not too far away
  if (last_added_node_ &&
      distance_since_last_added_node_ < chain_distance_threshold_*0.5 &&
      constraints[0].src != *last_added_node_ &&
      g.hasOptimizedPose(*last_added_node_)) {
    const tf::Pose last_pose = g.getOptimizedPose(*last_added_node_);
    if (last_pose.getOrigin().distance(opt_pose_estimate.getOrigin()) <
        local_window_size_) {
      constraints.push_back(makeConstraint(g, n, *last_added_node_, res, scan));
      ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Also adding constraint"
                              " to last added node " << *last_added_node_);
    }
  }
  return constraints;
}




msg::GraphConstraint ScanMatchConstraints::makeConstraint (const pg::ConstraintGraph& g, const unsigned n,
                                                           const ksm::ScanMatchResult& res, const pg::NodeSet& nodes,
                                                           LocalizedScan::ConstPtr scan) const
{
  const tf::Pose base_pose = util::toPose(res.pose);
  const gm::Point new_barycenter = util::transformPoint(base_pose, scans_.get(n)->barycenter);
  const unsigned closest = nodeWithNearestBarycenter(g, nodes, new_barycenter);
  //ROS_DEBUG_STREAM_NAMED ("scan_match_constraints", "Matched node " << n << "; against " << util::toString(nodes) <<
  //                        "; closest node was " << closest << " and response was " << res.response);

  return makeConstraint(g, n, closest, res, scan);
}


msg::GraphConstraint ScanMatchConstraints::makeConstraint (const pg::ConstraintGraph& g, const unsigned n,
                                                           const unsigned ref, const ksm::ScanMatchResult& res, 
                                                           LocalizedScan::ConstPtr scan) const
{
  const tf::Pose base_pose = util::toPose(res.pose);
  
  // The scan matching gave us an estimate of the base pose at the time the scan was taken.
  // But our constraint should refer to the base pose at the time of node creation.
  const tf::Pose node_in_laser_frame = util::toPose(scan->sensor_pose).inverse();
  const tf::Pose node_pose = base_pose*laser_offset_*node_in_laser_frame;

  msg::GraphConstraint constraint;
  constraint.src = ref;
  constraint.dest = n;
  constraint.constraint.pose = util::toPose(util::relativePose(node_pose, g.getOptimizedPose(ref)));

  /// \todo Does the precision matrix need to be rotated given that we've applied base_to_node?
  const Eigen::Matrix3f prec2D = res.cov.inverse();
  constraint.constraint.precision = util::makePrecisionMatrix(prec2D(0,0), prec2D(1,1), prec2D(0,1), prec2D(2,2));

  return constraint;
}



unsigned ScanMatchConstraints::nodeWithNearestBarycenter (const pg::ConstraintGraph& g,
                                                            const pg::NodeSet& nodes, const gm::Point& p) const
{
  boost::optional<unsigned> closest;
  double dist = -42.42;
  BOOST_FOREACH (const unsigned n2, nodes) {
    const tf::Pose node_pose = g.getOptimizedPose(n2);
    const gm::Point& b = util::transformPoint(node_pose, scans_.get(n2)->barycenter);
    const double dx=b.x-p.x;
    const double dy=b.y-p.y;
    const double new_dist = dx*dx + dy*dy;
    if (!closest || new_dist < dist) {
      dist = new_dist;
      closest = n2;
    }
  }
  ROS_ASSERT(closest);
  return *closest;
}




/************************************************************
 * Chains
 ***********************************************************/


MaybeNode ScanMatchConstraints::previousNode (const unsigned n) const
{
  MaybeNode prev;
  if (before_.hasData(n))
    prev = before_.get(n)->data;
  return prev;
}

MaybeNode ScanMatchConstraints::nextNode (const unsigned n) const
{
  MaybeNode next;
  if (after_.hasData(n))
    next = after_.get(n)->data;
  return next;
}

void ScanMatchConstraints::addSequenceLink (const unsigned n1, const unsigned n2)
{
  std_msgs::UInt64 m1, m2;
  m1.data = n1;
  m2.data = n2;
  before_.set(n2, m1);
  after_.set(n1, m2);
  ROS_DEBUG_STREAM_NAMED ("scan_chain_constraints", "Adding sequence link from " << n1 << " to " << n2);
}

Chain ScanMatchConstraints::getChain (const unsigned n, const pg::NodeSet& nodes,
                                      const pg::NodeSet& forbidden) const
{
  using util::contains;
  if (contains(forbidden, n) || !contains(nodes, n))
    return Chain();
  
  Chain before, after;
  {
    unsigned current = n;
    while (true) {
      MaybeNode prev = previousNode(current);
      if (prev && contains(nodes, *prev) && !contains(forbidden, *prev)) {
        before.push_back(*prev);
        current = *prev;
      }
      else
        break;
    }
  }

  {
    unsigned current = n;
    while (true) {
      MaybeNode next = nextNode(current);
      if (next && contains(nodes, *next) && !contains(forbidden, *next)) {
        after.push_back(*next);
        current = *next;
      }
      else
        break;
    }
  }

  Chain chain(before.size() + after.size() + 1);
  copy(before.rbegin(), before.rend(), chain.begin());
  chain[before.size()] = n;
  copy(after.begin(), after.end(), chain.begin() + (before.size() + 1));
  return chain;  
}

  
} // namespace laser_slam






