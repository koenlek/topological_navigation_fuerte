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
#include <laser_slam/scan_match_localization.h>
#include <graph_mapping_utils/utils.h>
#include <pose_graph/graph_search.h>
#include <kidnapped_robot/SavePlace.h>
#include <kidnapped_robot/MatchRequest.h>
#include <boost/foreach.hpp>


namespace laser_slam
{

namespace pg=pose_graph;
namespace util=graph_mapping_utils;
namespace msg=graph_mapping_msgs;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace ksm=karto_scan_matcher;
namespace kr=kidnapped_robot;

using std::string;

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



/************************************************************
 * Initialization
 ***********************************************************/


ScanMatchLocalizer::ScanMatchLocalizer(const ros::NodeHandle param_nh,
                                       boost::shared_ptr<tf::TransformListener> tf) :
  param_nh_(param_nh), update_rate_(util::getParam<double>(param_nh_, "update_rate", 5.0)),
  scan_match_proportion_(util::getParam<double>(param_nh_, "scan_match_proportion", 1.0)),
  max_nbd_size_(util::getParam<double>(param_nh_, "scan_match_nbd_size", 20)),
  fixed_frame_(util::searchParam<string>(param_nh_, "fixed_frame")),
  base_frame_(util::searchParam<string>(param_nh_, "base_frame")),
  opt_frame_(util::searchParam<string>(param_nh_, "optimization_frame", "graph_optimization")),
  window_size_(util::getParam<double>(param_nh_, "window_size", 5.0)),
  odom_noise_(util::getParam<double>(param_nh_, "odom_noise", 0.0)),
  match_radius_(util::getParam<double>(param_nh_, "match_radius", 0.3)),
  match_resolution_(util::getParam<double>(param_nh_, "match_resolution", 0.01)),
  angular_resolution_(util::getParam<double>(param_nh_, "angular_resolution", 0.5)),
  do_global_localization_(util::getParam<bool>(param_nh_, "do_global_localization", false)),
  initialized_(false), optimize_flag_(true), match_count_(0), tf_(tf), 
  scans_("graph_mapping", "scans"), ff_poses_("graph_mapping", "fixed_frame_poses"),
  loc_pub_(nh_.advertise<msg::LocalizationDistribution>("graph_localization", 10)),
  image_save_pub_(nh_.advertise<kr::SavePlace>("save_image", 10)),
  match_req_pub_(nh_.advertise<kr::MatchRequest>("match", 10)),
  init_pose_sub_(nh_.subscribe("initialpose", 5, &ScanMatchLocalizer::setInitialPose, this)),
  scan_sub_(nh_.subscribe("scan", 1, &ScanMatchLocalizer::storeScan, this)),
  match_sub_(nh_.subscribe("transform_matches", 100, &ScanMatchLocalizer::matchCB, this)),
  diff_sub_(boost::bind(&ScanMatchLocalizer::diffCB, this, _1, _2)),
  get_poses_client_(nh_.serviceClient<msg::GetPoses>("get_node_poses")),
  update_timer_(nh_.createTimer(util::duration(update_rate_), &ScanMatchLocalizer::update, this))
{
  Lock l(mutex_);

  // Look up and store the transform between the base and laser frames, which is assumed fixed
  const string laser_frame = util::searchParam<string>(param_nh_, "laser_frame");
  while (ros::ok()) {
    ros::Time t = ros::Time::now();
    if (tf_->waitForTransform(base_frame_, laser_frame, t, ros::Duration(5.0))) {
      gm::PoseStamped id, laser_pose;
      id.pose.orientation.w = 1.0;
      id.header.stamp = t;
      id.header.frame_id = laser_frame;
      tf_->transformPose(base_frame_, id, laser_pose);
      laser_offset_ = util::toPose(laser_pose.pose); 
      break;
    }
    ROS_INFO_STREAM ("Waiting for transform between " << base_frame_ << " and " << laser_frame);
  }
  
  initialized_ = ros::ok();
  ROS_DEBUG_NAMED ("init", "Initialized scan match localizer");
}

/************************************************************
 * Utilities
 ***********************************************************/


bool ScanMatchLocalizer::closerTo (const pg::ConstraintGraph& g,
                                   const tf::Vector3& barycenter,
                                   const unsigned n1, const unsigned n2) const
{
  if (!g.hasOptimizedPose(n2))
    return true;
  else if (!g.hasOptimizedPose(n1))
    return false;
  
  const tf::Pose p1 = g.getOptimizedPose(n1);
  const tf::Pose p2 = g.getOptimizedPose(n2);
  const tf::Vector3 b1 = p1*util::toPoint(scans_.get(n1)->barycenter);
  const tf::Vector3 b2 = p2*util::toPoint(scans_.get(n2)->barycenter);
  return (barycenter.distance2(b1) < barycenter.distance2(b2));
}


unsigned ScanMatchLocalizer::closestNode (const pg::ConstraintGraph& g,
                                            const pg::NodeSet& nodes,
                                            const tf::Vector3& p) const
{
  ROS_DEBUG_STREAM_NAMED ("localization_matching",
                          "Looking for closest node among nodes " <<
                          util::toString(nodes));
  pg::NodeSet::const_iterator closest =
    min_element(nodes.begin(), nodes.end(),
                boost::bind(&ScanMatchLocalizer::closerTo, this,
                            boost::ref(g), boost::ref(p), _1, _2));
  return *closest;
}

void ScanMatchLocalizer::recomputeOptimizedPoses (pg::ConstraintGraph* g, const pg::NodeSet& comp)
{
  BOOST_FOREACH (const unsigned n, comp)
  {
    if (!util::contains(opt_poses_, n))
      optimize_flag_ = true;
  }
  if (optimize_flag_)
  {
    optimizeGraph(g, &opt_poses_, comp, get_poses_client_);
    optimize_flag_ = false;
  }
  g->setOptimizedPoses(opt_poses_);
}



/************************************************************
 * Callbacks
 ***********************************************************/

void ScanMatchLocalizer::storeScan (sm::LaserScan::ConstPtr scan)
{
  Lock l(mutex_);
  if (initialized_) {
    last_scan_ = scan;
    initialization_scan_ = scan;
  }
}

void ScanMatchLocalizer::diffCB (boost::optional<const msg::ConstraintGraphDiff&> diff,
                                 const pg::ConstraintGraph& graph)
{
  Lock l(mutex_);
  if (!diff || (diff && diff->new_edges.size() > 0))
    optimize_flag_ = true;
  else {
    ROS_ASSERT (diff->new_node_timestamps.size()==1 && diff->new_nodes.size()==1);
    kr::SavePlace m;
    m.stamp = diff->new_node_timestamps[0];
    m.id = diff->new_nodes[0].id;
    image_save_pub_.publish(m);
    ROS_DEBUG_STREAM_NAMED ("place_rec", "Saving image: " << m);
  }
}


void ScanMatchLocalizer::matchCB (const kr::MatchResult& match)
{
  Lock l(mutex_);
  // The place rec node publishes all good matches in best-to-worst order
  // So we only save this match if no match has been saved yet this time round
  if (!match_result_) {
    match_result_ = match;
    ROS_INFO_STREAM ("Match result: " << match);
    const tf::Pose p(tf::Quaternion(match.transform.rotation.x, match.transform.rotation.y,
                                  match.transform.rotation.z, match.transform.rotation.w),
                     tf::Vector3(match.transform.translation.x, match.transform.translation.y,
                               match.transform.translation.z));
    const unsigned n(match.match_id);
    pg::ConstraintGraph g = diff_sub_.getCurrentGraph();
    const pg::NodeSet comp = pg::componentContaining(g, n);
    recomputeOptimizedPoses(&g, comp);
    const tf::Pose init_estimate = g.getOptimizedPose(n)*p;
    adjustInitialPose(init_estimate, g, comp, 2.0);
  }
}

/// \todo needs to be updated!
void ScanMatchLocalizer::setInitialPose (const gm::PoseWithCovarianceStamped& m)
{
  Lock l(mutex_);
  while (!tf_->waitForTransform(opt_frame_, m.header.frame_id, ros::Time(), ros::Duration(3.0))) 
    ROS_INFO_STREAM ("Waiting for transform between " << opt_frame_ << " and " << m.header.frame_id);
  gm::PoseStamped pose, opt_pose;
  pose.header = m.header;
  pose.pose = m.pose.pose;
  tf_->transformPose(opt_frame_, pose, opt_pose);
  const tf::Pose init_estimate = util::toPose(opt_pose.pose);
  pg::ConstraintGraph g = diff_sub_.getCurrentGraph();
  const pg::NodeSet comp = largestComp(g);
  ROS_INFO_STREAM ("Largest component has size " << comp.size());

  recomputeOptimizedPoses(&g, comp);
  adjustInitialPose(init_estimate, g, comp, 5.0);
}


void ScanMatchLocalizer::adjustInitialPose (const tf::Pose& init_estimate,
                                            const pg::ConstraintGraph& g, const pg::NodeSet& comp,
                                            const double global_loc_radius)
{


  ros::Time t;
  tf::Pose corrected_pose;
  if (initialization_scan_) {
    MatcherPtr matcher(new ksm::KartoScanMatcher(*initialization_scan_, util::projectToPose2D(laser_offset_), 5.0, 0.05));
    pg::NodeSet nearby;
    const tf::Vector3& pos = init_estimate.getOrigin();
    BOOST_FOREACH (const unsigned n, comp) {
      if (g.getOptimizedPose(n).getOrigin().distance(pos) < 5.0)
        nearby.insert(n);
    }
    boost::optional<tf::StampedTransform> prev_ff_pose =
      fixedFramePoseAt(initialization_scan_->header.stamp);
    const ksm::ScanMatchResult res = globalLocalization(g, matcher, nearby, scans_,
                                                        *initialization_scan_, laser_offset_, global_loc_radius, 0.2,
                                                        pos.x(), pos.y(), pos.x(), pos.y());

    // Further correction for odometry if possible
    t = ros::Time::now();
    boost::optional<tf::StampedTransform> current_ff_pose =
      fixedFramePoseAt(t);
    if (prev_ff_pose && current_ff_pose) 
      corrected_pose = util::toPose(res.pose)*util::relativePose(*current_ff_pose, *prev_ff_pose);
    else
      corrected_pose = util::toPose(res.pose);
  }
  else {
    corrected_pose = init_estimate;
    t = initialization_scan_->header.stamp;
    ROS_WARN ("Don't have an initialization scan; using init estimate without correcting.");
  }

  const unsigned closest = closestNode(g, g.allNodes(), corrected_pose.getOrigin());
  last_loc_.reset(new msg::LocalizationDistribution());
  last_loc_->stamp = t;
  last_loc_->samples.resize(1);
  last_loc_->samples[0].header.frame_id = util::nodeFrame(closest);
  last_loc_->samples[0].pose = util::toPose(util::relativePose(corrected_pose, g.getOptimizedPose(closest)));
  ROS_DEBUG_STREAM_NAMED ("localization", "Initialized localization to " << last_loc_->samples[0]);
}


/************************************************************
 * Update loop
 ***********************************************************/

void ScanMatchLocalizer::updateUsingFixedFrame ()
{
  const ros::Time t = ros::Time::now();
  const ros::Time& t_prev = last_loc_->stamp;
  const ros::Time t_loc = t_prev + (t-t_prev)*0.5;
  ROS_ASSERT_MSG (last_loc_->samples.size()==1, "Assumption of deterministic localization violated.");
  const gm::PoseStamped& l = last_loc_->samples[0];
  const unsigned ref = util::refNode(l);

  // Figure out the displacement in the fixed frame since the last localization
  gm::PoseStamped current;
  current.pose.orientation.w = 1.0;
  current.header.stamp = t_loc;
  current.header.frame_id = base_frame_;
  const util::MaybePose disp =
    util::waitAndTransform(*tf_, current, base_frame_, t_prev, fixed_frame_,
                           util::duration(update_rate_)*0.5);
  if (!disp)
  {
    ROS_WARN_STREAM ("Skipping localization as couldn't get transform between "
                     << t_loc << " and " << t_prev);
    return;
  }

  const tf::Pose prev_offset = util::toPose(last_loc_->samples[0].pose);
  
  // Important: since the pointer is shared across ros subscription
  last_loc_.reset(new msg::LocalizationDistribution()); 
  
  last_loc_->stamp = t_loc;
  last_loc_->samples.resize(1);
  last_loc_->samples[0].pose = util::transformPose(prev_offset, disp->pose);
  last_loc_->samples[0].header.frame_id = util::nodeFrame(ref);

  loc_pub_.publish(last_loc_);

}



void ScanMatchLocalizer::update (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  if (!initialized_) {
    ROS_INFO_THROTTLE_NAMED (3, "localization", "Waiting for initialization");
    return;
  }

  if (!last_scan_ || match_count_++ < 1/scan_match_proportion_) {

    // Case 1: No scans yet
    if (!last_loc_) 
      ROS_INFO_THROTTLE_NAMED(1, "localization", "Waiting for initial scan");

    // Case 2: have last loc, but no recent scan; use fixed frame transform
    else 
      updateUsingFixedFrame();
  }

  else {
    const ros::Time t = last_scan_->header.stamp;

    if (!matcher_) {
      gm::Pose2D offset = util::projectToPose2D(laser_offset_);
      matcher_.reset(new ksm::KartoScanMatcher(*last_scan_, offset,
                                               match_radius_,
                                               match_resolution_));
    }
    pg::ConstraintGraph g = diff_sub_.getCurrentGraph();

  
    if (!last_loc_) {
      // Case 3: startup with empty graph; return
      if (g.allNodes().empty()) {
        ROS_INFO_THROTTLE_NAMED (2, "localization",
                                 "Waiting for nonempty graph");
        return;
      }

      // Case 4: start up with single node; set localization to that node
      else if (g.allNodes().size() == 1) {
        last_loc_.reset(new msg::LocalizationDistribution());
        last_loc_->stamp = t;
        last_loc_->samples.resize(1);
        last_loc_->samples[0].pose.orientation.w = 1.0;
        last_loc_->samples[0].header.frame_id = util::nodeFrame(1);
        loc_pub_.publish(last_loc_);
      }

      else {
        ROS_INFO_THROTTLE_NAMED (2, "localization",
                                 "Waiting for initial localization for graph "
                                 " with %zu nodes", g.allNodes().size());
        const ros::Time t = ros::Time::now();
        if (!last_match_request_time_ ||
            t > *last_match_request_time_ + ros::Duration(3.0)) {
          ROS_DEBUG_STREAM_NAMED ("place_rec", "Sending match request");
          last_match_request_time_ = t;
          kr::MatchRequest req;
          req.stamp = t;
          match_req_pub_.publish(req);
          match_result_.reset(); // Make it so the next match we receive gets saved
        }
      }
    }

    else {
      const gm::PoseStamped& l = last_loc_->samples[0];
      const unsigned ref = util::refNode(l);

      // Recompute the relevant optimized poses if necessary
      recomputeOptimizedPoses(&g, pg::componentContaining(g, ref));

      // Case 5: we have a last localization and a new scan to update it with
      const ros::Time& t_prev = last_loc_->stamp;
      msg::LocalizationDistribution::Ptr new_loc(new msg::LocalizationDistribution());

      // Figure out the displacement in the fixed frame since the last localization
      gm::PoseStamped current;
      current.pose.orientation.w = 1.0;
      current.header.stamp = t;
      current.header.frame_id = base_frame_;
      util::MaybePose disp =
        util::waitAndTransform(*tf_, current, base_frame_, t_prev, fixed_frame_,
                               util::duration(update_rate_)*0.5);
      if (!disp)
      {
        ROS_WARN_STREAM ("Skipping localization as couldn't get transform between time " << t << " and " << t_prev);
        return;
      }

      ROS_ASSERT_MSG (last_loc_->samples.size()==1, "Assumption of deterministic localization violated.");
      new_loc->stamp = t;
      new_loc->samples.resize(1);

      ROS_ASSERT (g.hasOptimizedPose(ref)); // We optimized it earlier
      const tf::Pose old_pose =
        util::absolutePose(g.getOptimizedPose(ref), l.pose);
      const tf::Pose init_estimate = old_pose*util::toPose(disp->pose);
      const pg::OptimizedDistancePredicate
        pred(g, init_estimate.getOrigin(), window_size_);
      const pg::NodeSet nearby_nodes = pg::filterNearbyNodes(g, ref, pred);
      if (nearby_nodes.empty()) 
      {
        ROS_WARN ("Reverting to fixed frame update because "
                  "currently too far from any graph nodes");
        updateUsingFixedFrame();
      }
      else
      {
        /*
        try 
        {*/
          const pg::NodeSet nbd =
            util::sampleSubset(nearby_nodes, max_nbd_size_);
          const ksm::ScanMatchResult res =
            scanMatchNodes(g, matcher_, nbd, scans_, *last_scan_,
                           init_estimate, laser_offset_);
          match_count_=0;
      
          const tf::Pose new_pose = util::toPose(res.pose);

          // Slightly wrong since barycenter is in laser frame not base frame
          const tf::Vector3& b = new_pose*util::barycenter(*last_scan_);
          const unsigned closest = closestNode(g, nearby_nodes, b);

          new_loc->samples[0].header.frame_id = util::nodeFrame(closest);
          const tf::Pose closest_pose = g.getOptimizedPose(closest);
          new_loc->samples[0].pose =
            util::toPose(util::relativePose(new_pose, closest_pose));
          last_loc_ = new_loc;
          loc_pub_.publish(last_loc_);
          //}
        /*
        catch (karto::Exception& e) {
          ROS_WARN_STREAM ("Received Karto exception in scan match: " <<
                           e.GetErrorMessage() <<
                           "; reverting to fixed frame update.");
          updateUsingFixedFrame();
          return;
        }
        */
      }
    }

    last_scan_.reset();
  }
  
}



boost::optional<tf::StampedTransform>
ScanMatchLocalizer::fixedFramePoseAt (const ros::Time& t)
{
  return util::getTransform(*tf_, fixed_frame_, base_frame_,
                            t, util::duration(update_rate_));
}


} // namespace laser_slam

