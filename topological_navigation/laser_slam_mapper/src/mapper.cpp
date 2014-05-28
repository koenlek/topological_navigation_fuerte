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
 * Node that maintains a topological map
 *
 * \author Bhaskara Marthi
 */

#include <topological_map_2d/ros_conversion.h>
#include <pose_graph/constraint_graph.h>
#include <pose_graph/diff_subscriber.h>
#include <pose_graph/graph_db.h>
#include <topological_nav_msgs/SwitchGrid.h>
#include <pcl/ros/conversions.h>
#include <octomap_ros/OctomapROS.h>
#include <pose_graph/graph_search.h>
#include <laser_slam/LocalizedScan.h>
#include <graph_mapping_utils/utils.h>
#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <graph_mapping_msgs/GetPoses.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>


namespace laser_slam_mapper
{

namespace pg=pose_graph;
namespace gmu=graph_mapping_utils;
namespace mr=mongo_ros;
namespace gmm=graph_mapping_msgs;
namespace vm=visualization_msgs;
namespace nm=nav_msgs;
namespace tmap=topological_map_2d;
namespace ls=laser_slam;
namespace gu=occupancy_grid_utils;
namespace gm=geometry_msgs;
namespace msg=topological_nav_msgs;
namespace om=octomap;
namespace omr=octomap;
namespace sm=sensor_msgs;

using std::map;
using std::set;
using std::string;
using std::pair;
using std::vector;

using gmu::toString;

typedef boost::optional<const gmm::ConstraintGraphDiff&> OptionalDiff;
typedef boost::mutex::scoped_lock Lock;
typedef map<unsigned, tf::Pose> NodePoses;
typedef set<unsigned> Nodes;
typedef set<unsigned> Grids;
typedef map<unsigned, unsigned> NodeGridMap;
typedef vector<string> StringVec;

/************************************************************
 * Node class def
 ***********************************************************/

class Mapper
{
public:

  /// Initialize node
  Mapper ();
  
private:

  /****************************************
   * Entry points
   ****************************************/

  /// Periodically update and publish graph
  void updateLoop (const ros::TimerEvent& e);

  /// Visualize graph
  void visualize (const ros::TimerEvent& e);

  /// Receive a diff to the pose graph
  void diffCB (OptionalDiff diff, const pg::ConstraintGraph& g);

  /// Receive a localization wrt slam graph
  void locCB (const gmm::LocalizationDistribution& dist);

  /// Publish reference transforms over ros
  void publishRefTransforms(const ros::TimerEvent& e);

  /// Switch grid based on external request
  bool switchGrid (msg::SwitchGrid::Request& req,
                   msg::SwitchGrid::Response& resp);

  /****************************************
   * Modifying functions
   ****************************************/

  /// \post Ensure that there is a grid g that covers node n
  void ensureNodeCovered (unsigned n);

  /// \post There's a new grid centered at \a n
  void addGrid (unsigned n);

  /// Save local grid to db
  void saveGrid (unsigned g);

  /// \post An edge exists between \a g and \a g2 if they're
  /// close enough, and the transform on the edge is updated based
  /// on the current optimized poses
  void updateEdgeInfo (unsigned g, unsigned g2);

  /// \post Possible edges to all topologically close nodes to \a g
  /// have been updated
  void updateEdgeInfo (unsigned g);


  /****************************************
   * Non-modifying functions
   ****************************************/
  
  /// \retval Pose of node \a n wrt grid \a g
  tf::Pose poseOnGrid (unsigned n, unsigned g) const;

  /// \retval Optimized poses of some nodes
  NodePoses optimizePoses (const pg::NodeSet& comp) const;

  /// \retval Is node \a n covered by (near center of) grid centered at \a g?
  /// If so, return g, else return 0.
  unsigned coveredBy (unsigned n, unsigned g, double r) const;

  /// \retval Is node \a n covered by one of the grids?  If so, return
  /// the grid id.  Else return 0.
  unsigned coveredBy (unsigned n, const Grids& g, double r) const;

  /// Compute local occupancy grid
  nm::OccupancyGrid::ConstPtr computeLocalGrid (unsigned g) const;

  /// Compute local octomap
  om::OcTree computeLocalOctomap (unsigned g) const;

  /// \retval Nodes that are on a given local grid
  Nodes nodesOnGrid (unsigned g) const;

  /// \retval Nodes which are topologically close to the grid and
  /// whose scan barycenters are near the center
  Nodes overlayingNodes (unsigned g) const;

  /// Transform from pose graph node frame to local grid frame
  tf::Pose nodeGridTransform (unsigned n, unsigned g) const;

  /// Nodes that are topologically close in slam graph
  Nodes topologicallyCloseNodes (unsigned g, const double r) const;

  /// Grids that are currently relevant to update.  This is currently
  /// just the current reference grid we're localized wrt
  Grids currentRelevantGrids () const;
  


  /****************************************
   * Ros broadcast
   ****************************************/

  /// Publish graph over ros
  void publishGraph ();

  // Save current grid
  void saveCurrentGrid ();

  /// Visualize the boundaries of all grids
  void visualizeGridBoundaries ();

  /// Visualize the grid we're currently localized wrt
  void visualizeCurrentGrid ();

  /// Visualize edges between grids of the topological map
  void visualizeEdges ();
  

  /****************************************
   * Parameters
   ****************************************/

  // Ros namespace from which we'll read parameters
  ros::NodeHandle param_nh_;

  // The side length of a local grid
  const double grid_size_;

  // Governs how often new grids are created.  1 will create a new grid for each
  // node, while 0 will create a new grid only if node is off existing grids.
  const double grid_overlap_;

  // Resolution of local grids
  const double resolution_;

  // Frame in which optimized poses are defined
  const string ref_frame_;

  // Rate at which graph is updated
  const double update_rate_;

  // Radius of robot is used to do additional obstacle clearing if positive
  const double robot_radius_;

  /****************************************
   * State
   ****************************************/

  // The pose graph from graph_slam
  pg::ConstraintGraph pose_graph_;
  
  // The topological map itself
  tmap::TopologicalMap tmap_;

  // Does the topological map possibly need to be updated?
  bool update_flag_;

  // The most recent localization
  boost::optional<gm::PoseStamped> last_loc_;

  // Optimized poses
  NodePoses opt_poses_;

  // Pending nodes to be processed
  Nodes pending_nodes_;

  // Nodes that have a corresponding grid centered at them
  Nodes grid_nodes_;

  // Current grid
  boost::optional<unsigned> current_grid_;

  // Map each node to a grid that covers it
  NodeGridMap covering_grids_;

  // Counter to generate unique edge ids
  unsigned next_edge_id_;

  // Last time a diff was received
  ros::Time last_diff_time_;

  // Last time each grid was saved
  map<unsigned, ros::Time> grid_save_times_;


  /****************************************
   * Associated objects
   ****************************************/

  // Mutable because const operations may need to acquire a lock
  mutable boost::mutex mutex_;

  // Node handle for ros communications
  ros::NodeHandle nh_;

  // Tf listener
  tf::TransformListener tf_;

  // Transform broadcaster for grid frames
  tf::TransformBroadcaster tfb_;

  // Stored scans at graph slam nodes
  pg::CachedNodeMap<ls::LocalizedScan> scans_;

  // Stored clouds at graph slam nodes
  pg::CachedNodeMap<sm::PointCloud2> clouds_;

  // Collection for storing local grids in topo_db
  mr::MessageCollection<nm::OccupancyGrid> grids_;

  // Subscription to graph localization
  ros::Subscriber loc_sub_;

  // Subscribe to pose graph via diffs
  pg::DiffSubscriber diff_sub_;

  // Publish visualization markers
  ros::Publisher marker_pub_;

  // Publish local grids
  ros::Publisher grid_pub_;

  // Publish graph over ros
  ros::Publisher graph_pub_;

  // Publish graph localization over ros
  ros::Publisher loc_pub_;

  // Publish octomap for visualization
  ros::Publisher octomap_pub_;

  // Service handler for external switch grids requests
  ros::ServiceServer switch_grid_srv_;

  // Client to get optimized poses
  // Mutable because calling it doesn't change state
  mutable ros::ServiceClient get_poses_client_;

  // Timer to call updateLoop
  ros::Timer update_timer_;

  // Timer that calls visualize
  ros::Timer vis_timer_;

  // Timer to publish transforms
  ros::Timer transform_pub_timer_;

  
};


/************************************************************
 * Initialization
 ***********************************************************/


StringVec gridIndexes ()
{
   StringVec fields;
   fields.push_back("id");
   return fields;
}

Mapper::Mapper () :
  // Params
  param_nh_("~"), 
  grid_size_(gmu::getParam(param_nh_, "grid_size", 15.0)),
  grid_overlap_(gmu::getParam(param_nh_, "grid_overlap", 0.5)),
  resolution_(gmu::getParam(param_nh_, "resolution", 0.025)),
  ref_frame_(gmu::getParam<string>(param_nh_, "reference_frame")),
  update_rate_(gmu::getParam<double>(param_nh_, "update_rate", 0.4)),
  robot_radius_(gmu::getParam<double>(param_nh_, "robot_radius", -1)),

  // State
  update_flag_(true), next_edge_id_(1),

  // Other objects
  scans_("graph_mapping", "scans"), clouds_("graph_mapping", "clouds"),
  grids_("topological_navigation", "grids"), //KL: orig was grids_("topological_navigation", "grids", gridIndexes()),
  loc_sub_(nh_.subscribe("graph_localization", 5, &Mapper::locCB, this)),
  diff_sub_(boost::bind(&Mapper::diffCB, this, _1, _2)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 5)),
  grid_pub_(nh_.advertise<nm::OccupancyGrid>("local_grid", 5)),
  graph_pub_(nh_.advertise<msg::TopologicalGraph>("topological_graph", 3)),
  loc_pub_(nh_.advertise<gm::PoseStamped>("topological_localization", 5)),
  octomap_pub_(nh_.advertise<vm::MarkerArray>("visualization_marker_array", 5)),
  switch_grid_srv_(nh_.advertiseService("switch_local_grid",
                                         &Mapper::switchGrid, this)),
  get_poses_client_(nh_.serviceClient<gmm::GetPoses>("get_node_poses")),
  update_timer_(nh_.createTimer(gmu::duration(update_rate_),
                                &Mapper::updateLoop, this)),
  vis_timer_(nh_.createTimer(ros::Duration(2.0), &Mapper::visualize, this)),
  transform_pub_timer_(nh_.createTimer(ros::Duration(0.1),
                                       &Mapper::publishRefTransforms, this))
{
  ROS_DEBUG_STREAM_NAMED ("init", "Mapper initialization complete");\
}


/************************************************************
 * Updates
 ***********************************************************/

// Call the mapper service to return optimized poses of a set
// of nodes
NodePoses Mapper::optimizePoses (const pg::NodeSet& comp) const
{
  gmm::GetPoses srv;
  BOOST_FOREACH (const unsigned n, comp) 
    srv.request.nodes.push_back(n);
  if (!get_poses_client_.call(srv))
    throw std::runtime_error("Service call to optimizer failed");
  NodePoses poses;
  unsigned i=0;
  BOOST_FOREACH (const unsigned n, srv.request.nodes) 
    poses[n] = gmu::toPose(srv.response.poses[i++]);
  return poses;  
}

void Mapper::updateLoop (const ros::TimerEvent& e)
{
  pg::NodeSet comp;
  pg::NodePoseMap opt_poses;
  {
    Lock l(mutex_);

    // Only update once we have a localization
    if (!last_loc_)
    {
      ROS_INFO_THROTTLE (5.0, "Topological mapper waiting for "
                         "initial localization");
      return;
    }

    // Optimize current graph component
    comp = pg::componentContaining(pose_graph_, gmu::refNode(*last_loc_));
  }
  opt_poses = optimizePoses(comp);

  Lock l(mutex_);
  opt_poses_ = opt_poses;
  pose_graph_.setOptimizedPoses(opt_poses_);
  // Ensure that each pending node has a grid that covers it
  BOOST_FOREACH (const unsigned n, pending_nodes_) 
  {
    if (!gmu::contains(opt_poses_, n))
    {
      ROS_DEBUG_STREAM_NAMED ("update_int", "No optimized pose yet for " <<
                              n << "; skipping it.");
      continue;
    }
    ROS_DEBUG_STREAM_NAMED ("update_int", "Processing pending node " << n);

    const double r = 1-grid_overlap_;
    if (!(current_grid_ && coveredBy(n, *current_grid_, r)) &&
        !coveredBy(n, grid_nodes_, r))
    {
      addGrid(n);
      updateEdgeInfo(n);
      saveGrid(n);
      covering_grids_[n] = n;
    }
    else
    {
      covering_grids_[n] = coveredBy(n, grid_nodes_, r);
    }
  }
  pending_nodes_.clear();

  // Update info for each grid we currently care about
  BOOST_FOREACH (const unsigned g, currentRelevantGrids()) 
    updateEdgeInfo (g);

  
  // Publish info over ros
  publishGraph();
  saveCurrentGrid();

  
}

unsigned Mapper::coveredBy (const unsigned n, const unsigned g,
                            const double r) const
{
  const tf::Pose& center = gmu::keyValue(opt_poses_, g);
  const tf::Pose& pose = gmu::keyValue(opt_poses_, n);
  const tf::Pose rel_pose = gmu::relativePose(pose, center);
  const double d = grid_size_*r*0.5;
  const double rx = rel_pose.getOrigin().x();
  const double ry = rel_pose.getOrigin().y();
  return (fabs(rx)<d && fabs(ry)<d) ? g : 0;
}


unsigned Mapper::coveredBy (const unsigned n, const set<unsigned>& grids,
                            const double r) const
{
  BOOST_FOREACH (const unsigned g, grids) 
  {
    if (coveredBy(n, g, r))
      return g;
  }
  return 0;
}

void Mapper::addGrid (const unsigned n)
{
  ROS_DEBUG_STREAM_NAMED ("update", "Adding grid centered at node " << n);
  grid_nodes_.insert(n);
  msg::TopologicalMapNode info;
  info.id = n;
  info.y_size = grid_size_;
  info.x_size = grid_size_;
  info.resolution = resolution_;
  tmap_.addNode(info);
}


void Mapper::updateEdgeInfo (const unsigned g, const unsigned g2)
{
  if (g!=g2 && gmu::contains(grid_nodes_, g2))
  {
    ROS_DEBUG_NAMED ("update_int", "Processing nearby nodes %u and %u",
                     g, g2);
    // At this point, g2 is a grid that's close to g.  
    pair<tmap::GraphEdge, bool> res =
      edge(tmap_.node(g), tmap_.node(g2), tmap_);
    if (res.second)
    {
      // If there's already an edge, just update the transform
      // based on the current optimized poses
      msg::TopologicalMapEdge& info = tmap_[res.first];
      const tf::Pose dest = gmu::keyValue(opt_poses_, info.dest);
      const tf::Pose src = gmu::keyValue(opt_poses_, info.src);
      tmap_.edgeInfo(tmap_[res.first].id).offset =
        gmu::toPose(gmu::relativePose(dest, src));
    }
    else
    {
      // Else add an edge if necessary
      const tf::Pose dest = gmu::keyValue(opt_poses_, g2);
      const tf::Pose src = gmu::keyValue(opt_poses_, g);
      const double d2 = dest.getOrigin().distance2(src.getOrigin());
      if (d2 < 2*grid_size_*grid_size_)
      {
        msg::TopologicalMapEdge e;
        e.id = next_edge_id_++;
        e.src = g;
        e.dest = g2;
        e.offset = gmu::toPose(gmu::relativePose(dest, src));
        tmap_.addEdge(e);
        ROS_DEBUG_NAMED("update", "Adding edge from %u to %u", g, g2);
      }
    }        
  }
}

void Mapper::updateEdgeInfo (const unsigned g)
{
  ROS_DEBUG_NAMED ("update_int", "Updating edge info for %u", g);
  BOOST_FOREACH (const unsigned n,
                 topologicallyCloseNodes(g, grid_size_*1.5))
  {
    updateEdgeInfo(g, n);
  }
}


Nodes Mapper::topologicallyCloseNodes (const unsigned g, const double r) const
{
  const tf::Vector3  pos = gmu::keyValue(opt_poses_, g).getOrigin();
  pg::OptimizedDistancePredicate pred(pose_graph_, pos, r);
  return filterNearbyNodes (pose_graph_, g, pred);
}

Grids Mapper::currentRelevantGrids () const
{
  Grids grids;
  if (current_grid_)
    grids.insert(*current_grid_);
  return grids;
}


/************************************************************
 * Callbacks
 ***********************************************************/

// Take in slam graph localization and publish a PoseStamped
// that represents pose on current local grid; update current grid
// if we're off the edge.
void Mapper::locCB (const gmm::LocalizationDistribution& dist)
{
  Lock l(mutex_);
  if (dist.samples.size()!=1)
    throw std::logic_error("Localization distribution was not deterministic ");
  last_loc_ = dist.samples[0];

  const unsigned ref_node = gmu::refNode(*last_loc_);
  if (!gmu::contains(opt_poses_, ref_node))
  {
    ROS_DEBUG_STREAM_NAMED ("localization", "Not updating localization due to"
                            " lack of optimized pose for " << ref_node);
    return;
  }
  if (!current_grid_ || !coveredBy(ref_node, *current_grid_, 1-grid_overlap_))
  {
    if (gmu::contains(covering_grids_, ref_node))
    {
      current_grid_ = covering_grids_[ref_node];
      ROS_DEBUG_STREAM_NAMED ("localization", "On new grid " << *current_grid_);
    }
  }

  gm::PoseStamped loc;
  loc.header.frame_id = tmap::gridFrame(*current_grid_);
  loc.header.stamp = dist.stamp;
  loc.pose = gmu::transformPose(nodeGridTransform(ref_node, *current_grid_),
                                last_loc_->pose);
  loc_pub_.publish(loc);
}


// When a diff comes in, incorporate it into the graph.  Also, add all new nodes
// to the pending_nodes_ queue  to be processed in the update loop, and 
void Mapper::diffCB (OptionalDiff diff,
                     const pg::ConstraintGraph& g)
{
  Lock l(mutex_);
  if (diff)
  {
    BOOST_FOREACH (const gmm::Node& n, diff->new_nodes) 
      pending_nodes_.insert(n.id);
    /// \todo deal with edges
  }
  else
  {
    BOOST_FOREACH (const unsigned n, g.allNodes()) 
      pending_nodes_.insert(n);
  }
  pose_graph_ = g;

  // Since the incoming graph doesn't have these set
  pose_graph_.setOptimizedPoses(opt_poses_); 

  last_diff_time_ = ros::Time::now();
}

bool Mapper::switchGrid (msg::SwitchGrid::Request& req,
                         msg::SwitchGrid::Response& resp)
{
  Lock l(mutex_);
  ROS_DEBUG_STREAM_NAMED ("localization",
                          "Switching to grid " << req.grid);
  current_grid_ = req.grid;
  return true;
}

/************************************************************
 * Local grids
 ***********************************************************/


// Not used any more
Nodes Mapper::nodesOnGrid (const unsigned g) const
{
  Nodes nodes;
  BOOST_FOREACH (const NodePoses::value_type& e, opt_poses_)
  {
    if (coveredBy(e.first, g, 1.0))
      nodes.insert(e.first);
  }
  return nodes;
}

Nodes Mapper::overlayingNodes (const unsigned g) const
{
  const Nodes potential_nodes = topologicallyCloseNodes(g, grid_size_*1.5);
  Nodes nodes;
  BOOST_FOREACH (const unsigned n, potential_nodes) 
  {
    ls::LocalizedScan::ConstPtr scan = scans_.get(n);
    const tf::Pose trans = nodeGridTransform(n, g);
    const tf::Point b = trans*gmu::toPoint(scan->barycenter);
    if ((fabs(b.x()) < grid_size_/2.0) &&
        (fabs(b.y()) < grid_size_/2.0))
      nodes.insert(n);
  }
  ROS_DEBUG_STREAM_NAMED ("overlay", "Overlaying grid " << g << " using "
                          << toString(nodes) << " (potential nodes were )"
                          << toString(potential_nodes));
    
  return nodes;
}

/// Transform from pose graph node frame to local grid frame
tf::Pose Mapper::nodeGridTransform (const unsigned n, const unsigned g) const
{
  const tf::Pose& grid_center_pose = gmu::keyValue(opt_poses_, g);
  const tf::Pose& node_pose = gmu::keyValue(opt_poses_, n);
  return grid_center_pose.inverseTimes(node_pose);
}


nm::OccupancyGrid::ConstPtr Mapper::computeLocalGrid (const unsigned g) const
{
  ROS_ASSERT_MSG (gmu::contains(grid_nodes_, g), "%u is not a grid node", g);
  const std::string frame = tmap::gridFrame(g);
  
  /// Set up data structure to do ray tracing
  nm::OccupancyGrid dims;
  dims.info.resolution = resolution_;
  dims.info.width = grid_size_/resolution_;
  dims.info.height = grid_size_/resolution_;
  
  const double r=grid_size_/2.0;
  dims.info.origin.position.x = -r;
  dims.info.origin.position.y = -r;
  dims.info.origin.orientation.w = 1.0;
  dims.header.frame_id = frame;
  gu::OverlayClouds overlay = gu::createCloudOverlay(dims, frame);

  /// Add the clouds
  BOOST_FOREACH (const unsigned n, overlayingNodes(g))
  {
    ls::LocalizedScan::ConstPtr stored_scan = scans_.get(n);
    gu::LocalizedCloud::Ptr cloud(new gu::LocalizedCloud());
    cloud->header.frame_id = frame;
    cloud->sensor_pose = gmu::transformPose(nodeGridTransform(n, g),
                                            stored_scan->sensor_pose);
    cloud->cloud = stored_scan->cloud;
    addCloud(&overlay, cloud);
  }

  /// Possibly clear out free space based on robot poses
  if (robot_radius_ > 0)
  {
    const tf::Transform world_to_grid = gmu::keyValue(opt_poses_, g).inverse();
    BOOST_FOREACH (const unsigned n, nodesOnGrid(g))
    {
      const tf::Pose node_pose = gmu::keyValue(opt_poses_, n);
      const gm::Point pos = gmu::toPoint(world_to_grid*node_pose.getOrigin());
      addKnownFreePoint(&overlay, pos, robot_radius_);
    }
  }

  return getGrid(overlay);                                                     
}

om::OcTree Mapper::computeLocalOctomap (const unsigned g) const
{
  ROS_ASSERT_MSG (gmu::contains(grid_nodes_, g), "%u is not a grid node", g);
  const std::string frame = tmap::gridFrame(g);
  
  // Set up octomap and params
  omr::OcTreeROS octomap(resolution_);

  // Add clouds
  BOOST_FOREACH (const unsigned n, overlayingNodes(g))
  {
    ROS_INFO ("Adding cloud for node %u", n);
    try
    {
      sm::PointCloud2::ConstPtr stored_cloud = clouds_.get(n);
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*stored_cloud, cloud);
    
      // Set cloud frame, params
      const tf::Pose node_pose = nodeGridTransform(n, g);
      const tf::Vector3 & pos = node_pose.getOrigin();
      const pcl::PointXYZ trans(pos.x(), pos.y(), pos.z());
      octomap.insertScan(cloud, pcl::PointXYZ(0,0,0), trans,
                         node_pose.getRotation());
    }
    catch (pg::DataNotFoundException& e)
    {
      ROS_WARN ("Didn't find stored cloud for %u when computing octomap", n);
    }
  }

  return octomap.octree;
}



/************************************************************
 * Ros publication/visualization
 ***********************************************************/

void Mapper::visualize (const ros::TimerEvent&)
{
  visualizeGridBoundaries();
  visualizeCurrentGrid();
  visualizeEdges();
}


vm::MarkerArray convertToMarkers (const om::OcTree& octree,
                                  const string& frame)
{
  vm::MarkerArray m;

  // Get dims
  double x, y, minZ, maxZ;
  octree.getMetricMin(x, y, minZ);
  octree.getMetricMax(y, y, maxZ);

  // each marker is a cube list storing all cubes at a given depth level
  m.markers.resize(octree.getTreeDepth());
  double lowestRes = octree.getResolution();

  // Loop over leaves and add to appropriate cube list
  for (om::OcTree::iterator it = octree.begin(),
         end = octree.end(); it != end; ++it)
  {
    if (octree.isNodeOccupied(*it))
    {
      // which array to store cubes in?
      int idx = int(log2(it.getSize() / lowestRes) +0.5);
      assert (idx >= 0 && unsigned(idx) < m.markers.size());

      gm::Point cubeCenter;
      cubeCenter.x = it.getX();
      cubeCenter.y = it.getY();
      cubeCenter.z = it.getZ();
      m.markers[idx].points.push_back(cubeCenter);
    }
  }

  // Set the remaining marker parameters
  for (unsigned i= 0; i < m.markers.size(); ++i)
  {
    double size = lowestRes * pow(2,i);
    m.markers[i].header.frame_id = frame;
    m.markers[i].header.stamp = ros::Time::now();
    m.markers[i].ns = "topological_octomap";
    m.markers[i].id = i;
    m.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    m.markers[i].scale.x = size;
    m.markers[i].scale.y = size;
    m.markers[i].scale.z = size;
    m.markers[i].color.b = m.markers[i].color.a = 1.0;
    m.markers[i].action = visualization_msgs::Marker::ADD;
  }

  return m;
}

void Mapper::visualizeCurrentGrid ()
{
  if (!last_loc_)
    return;

  Lock l(mutex_);
  const unsigned n = gmu::refNode(*last_loc_);
  if (!gmu::contains(covering_grids_, n))
  {
    ROS_DEBUG_STREAM_NAMED ("visualization", "Not visualizing current grid "
                            " because don't have covering grid for " << n);
    return;
  }
  grid_pub_.publish(computeLocalGrid(*current_grid_));
  /*octomap_pub_.publish(convertToMarkers(computeLocalOctomap(*current_grid_),
    ref_frame_));*/
}

void Mapper::visualizeGridBoundaries ()
{
  vm::Marker grids;
  grids.header.frame_id = ref_frame_;
  grids.header.stamp = ros::Time::now();
  grids.ns = "topological_map";
  grids.type = vm::Marker::LINE_LIST;
  grids.action = vm::Marker::ADD;
  grids.scale.x = 0.03;
  grids.color.r = 1.0;
  grids.color.g = 0.6;
  grids.color.b = 0.0;
  grids.color.a = 1.0;

  BOOST_FOREACH (const unsigned g, grid_nodes_) 
  {
    const double s = grid_size_*0.5;
    const tf::Pose origin = gmu::keyValue(opt_poses_, g);
    const int xm[] = {-1, 1, 1, -1};
    const int ym[] = {1, 1, -1, -1};
    for (int i=0; i<4; i++)
    {
      const int j = i ? i-1 : 3;
      const tf::Vector3  p1(s*xm[j], s*ym[j], 0);
      const tf::Vector3  p2(s*xm[i], s*ym[i], 0);
      grids.points.push_back(gmu::toPoint(origin*p1));
      grids.points.push_back(gmu::toPoint(origin*p2));
    }
  }
  marker_pub_.publish(grids);
}

void Mapper::visualizeEdges ()
{
  vm::Marker edges;
  edges.header.frame_id = ref_frame_;
  edges.header.stamp = ros::Time::now();
  edges.ns = "topological_map_edges";
  edges.type = vm::Marker::LINE_LIST;
  edges.action = vm::Marker::ADD;
  edges.scale.x = 0.03;
  edges.color.r = 1.0;
  edges.color.g = 0.0;
  edges.color.b = 0.0;
  edges.color.a = 1.0;
  BOOST_FOREACH (const unsigned g, grid_nodes_) 
  {
    const tf::Pose origin = gmu::keyValue(opt_poses_, g);
    BOOST_FOREACH (const tmap::GraphEdge e, out_edges(tmap_.node(g), tmap_)) 
    {
      if (tmap_[e].src == g)
      {
        edges.points.push_back(gmu::toPoint(origin.getOrigin()));
        edges.points.push_back(gmu::transformPoint(origin,
                                                   tmap_[e].offset.position));
      }
    }
  }
  marker_pub_.publish(edges);
}

void Mapper::saveGrid (const unsigned g)
{
  typedef mr::MessageWithMetadata<nm::OccupancyGrid>::ConstPtr GridWithMetadata;
  ROS_DEBUG_STREAM_NAMED ("grid", "Storing local grid " << g);
  nm::OccupancyGrid::ConstPtr grid = computeLocalGrid(g);

  mr::Query q = mr::Query("id", g);
  vector<GridWithMetadata> existing = grids_.pullAllResults(q, true);
  ROS_DEBUG_NAMED ("grid", "There were %zu existing grids", existing.size());
  if (existing.size()>0)
  {
    const unsigned num_removed = grids_.removeMessages(q);
    ROS_WARN_COND_NAMED (num_removed!=existing.size(), "grid",
                         "Removed %u grids instead of %zu", num_removed,
                         existing.size());
  }

  grids_.insert(*grid, mr::Metadata("id", g));
  grid_save_times_[g] = ros::Time::now();
}

void Mapper::publishGraph ()
{
  msg::TopologicalGraph::Ptr m;
  m = tmap::toMessage(tmap_);
  graph_pub_.publish(m);
}


void Mapper::publishRefTransforms (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  BOOST_FOREACH (const unsigned g, grid_nodes_) 
  {
    tf::StampedTransform tr(gmu::keyValue(opt_poses_, g), ros::Time::now(),
                            ref_frame_, tmap::gridFrame(g));
    tfb_.sendTransform(tr);
  }
}

void Mapper::saveCurrentGrid ()
{
  // Also store/update the current local grid
  if (current_grid_) {
    if (!gmu::contains(grid_save_times_, *current_grid_) ||
        gmu::keyValue(grid_save_times_, *current_grid_) < last_diff_time_) {
      saveGrid(*current_grid_);
      grid_pub_.publish(computeLocalGrid(*current_grid_));
    }
  }
}


} // namespace




int main (int argc, char** argv)
{
  ros::init(argc, argv, "topological_mapper");
  laser_slam_mapper::Mapper node;
  ros::spin();
  return 0;
}
