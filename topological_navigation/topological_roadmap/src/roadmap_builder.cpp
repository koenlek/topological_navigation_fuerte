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
 * Ros node that maintains a topological roadmap
 *
 * \author Bhaskara Marthi
 */



#include <topological_roadmap/roadmap.h>
#include <topological_roadmap/ros_conversion.h>
#include <graph_mapping_utils/ros.h>
#include <graph_mapping_utils/to_string.h>
#include <graph_mapping_utils/geometry.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <topological_map_2d/ros_conversion.h>
#include <mongo_ros/message_collection.h>
#include <topological_nav_msgs/RoadmapPath.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>

namespace topological_roadmap
{

namespace gmu=graph_mapping_utils;
namespace tmap=topological_map_2d;
namespace msg=topological_nav_msgs;
namespace gm=geometry_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using gmu::toString;
using std::string;
using std::vector;
using std::pair;
using std::map;
using boost::optional;

typedef boost::mutex::scoped_lock Lock;
typedef std::set<unsigned> Nodes;
typedef vector<gm::Point> PointVec;

/************************************************************
 * Node class
 ***********************************************************/

class RoadmapNode
{
public:
  RoadmapNode();

private:

  /****************************************
   * Entry points
   ****************************************/

  // Callback for topological graph
  void graphCB (const msg::TopologicalGraph& m);

  // Callback for topological localization
  void locCB (const gm::PoseStamped& l);

  // Callback for grid updates
  void gridUpdateCB (const std_msgs::String& m);

  // Callback for paths from navigation (for visualization)
  void pathCB (const msg::RoadmapPath& m);

  /****************************************
   * Const methods
   ****************************************/

  nm::OccupancyGrid::ConstPtr getGrid (const unsigned g) const;
  PointVec potentialWaypoints () const;

  /****************************************
   * Params
   ****************************************/

  // Node handle for parameters
  ros::NodeHandle param_nh_;

  // Navigation planning for edges assumes a circular robot
  // with this radius 
  const double robot_radius_;

  // How far apart waypoints are laid down
  const double waypoint_spacing_;

  // Frame into which data is transformed before visualizing
  const string vis_frame_;

  // Whether to visualize individual node ids
  const bool visualize_node_ids_;

  /****************************************
   * State
   ****************************************/

  // Most recent topological map
  tmap::TopologicalMap tmap_;

  // The roadmap being maintained
  Roadmap roadmap_;

  // Previous localization
  optional<gm::PoseStamped> last_loc_;

  // Size of local grids
  optional<double> grid_size_;

  // Last time each local grid was updated
  map<unsigned, ros::Time> grid_update_times_;

  // Last time roadmap was updated within a given grid
  map<unsigned, ros::Time> roadmap_update_times_;

  // Intended path from navigation (for visualization)
  vector<unsigned> path_;

  /****************************************
   * Associated objects
   ****************************************/

  // Single mutex for all state
  boost::mutex mutex_;

  // Node handle for ROS communications
  ros::NodeHandle nh_;

  // DB collection for grids
  mr::MessageCollection<nm::OccupancyGrid> grids_;

  // Tf listener
  tf::TransformListener tf_;

  // Timer to periodically publish roadmap
  ros::Timer vis_timer_;

  // Subscription to topological graph
  ros::Subscriber graph_sub_;

  // Subscription to topological localization
  ros::Subscriber loc_sub_;

  // Subscription to grid updates
  ros::Subscriber grid_sub_;

  // Subscription to paths
  ros::Subscriber path_sub_;

  // Publisher to visualize roadmap
  ros::Publisher vis_pub_;

  // Publisher for the roadmap itself
  ros::Publisher roadmap_pub_;

  // Debugging publisher to visualize inflated grid
  ros::Publisher inflated_grid_pub_;
};

/************************************************************
 * Initialization
 ***********************************************************/

RoadmapNode::RoadmapNode () :
  // Params
  param_nh_("~"),
  robot_radius_(gmu::getParam<double>(param_nh_, "robot_radius")),
  waypoint_spacing_(gmu::getParam<double>(param_nh_, "waypoint_spacing")),
  vis_frame_(gmu::getParam<string>(param_nh_, "visualization_frame", "map")),
  visualize_node_ids_(gmu::getParam<bool>(param_nh_, "visualize_node_ids", false)),

  // Objects
  grids_("topological_navigation", "grids"),
  graph_sub_(nh_.subscribe("topological_graph", 1, &RoadmapNode::graphCB,
                           this)),
  loc_sub_(nh_.subscribe("topological_localization", 5, &RoadmapNode::locCB,
                         this)),
  grid_sub_(nh_.subscribe("warehouse/topological_navigation/grids/inserts",
                          10000, &RoadmapNode::gridUpdateCB, this)),
  path_sub_(nh_.subscribe("roadmap_path", 5, &RoadmapNode::pathCB, this)),
  vis_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  roadmap_pub_(nh_.advertise<msg::TopologicalRoadmap>("topological_roadmap", 10)),
  inflated_grid_pub_(nh_.advertise<nm::OccupancyGrid>("inflated_grid", 10))
  
{
  ROS_DEBUG_STREAM_NAMED ("init", "Roadmap builder initialized");
}


/************************************************************
 * Graph update
 ***********************************************************/


optional<gm::Point> freePointNear (const nm::OccupancyGrid& grid,
                                   const double x0, const double y0,
                                   const double r)
{
  const nm::MapMetaData& geom=grid.info;
  const int cell_radius = floor(r/geom.resolution);
  gu::Cell center = gu::pointCell(geom, gmu::makePoint(x0, y0));
  optional<gm::Point> p;
  for (int dx=0; dx<=cell_radius; dx++)
  {
    for (int dy=0; dy<=cell_radius; dy++)
    {
      for (int sx=-1; sx<=1; sx+=2)
      {
        for (int sy=-1; sy<=1; sy+=2)
        {
          const gu::Cell c(center.x+dx*sx, center.y+dy*sy);
          if (grid.data[gu::cellIndex(geom, c)]==gu::UNOCCUPIED)
            return optional<gm::Point>(gu::cellCenter(geom, c));
        }
      }
    }
  }
  return optional<gm::Point>();
}

nm::OccupancyGrid::ConstPtr RoadmapNode::getGrid (const unsigned g) const
{
  typedef mr::MessageWithMetadata<nm::OccupancyGrid>::ConstPtr Grid;
  mr::Query q("id", g);
  vector<Grid> results = grids_.pullAllResults(q);
  if (results.size()>0)
    return results[0]; // Slicing the metadata away, which is fine
  else
    return nm::OccupancyGrid::ConstPtr();
}

PointVec RoadmapNode::potentialWaypoints () const
{
  PointVec points;
  const double r=*grid_size_/2.0;
  for (double x=-r+waypoint_spacing_/2.0; x<=r;
       x+=waypoint_spacing_)
  {
    for (double y=-r+waypoint_spacing_/2.0; y<=r;
         y+=waypoint_spacing_)
    {
      points.push_back(gmu::makePoint(x,y));
    }
  }
  return points;
}


void RoadmapNode::graphCB (const msg::TopologicalGraph& m)
{
  ROS_DEBUG_STREAM_NAMED ("graph_cb", "Entering graph callback");
  Lock l(mutex_);
  tmap_ = tmap::fromMessage(m);
  BOOST_FOREACH (const tmap::GraphVertex& v, vertices(tmap_)) 
  {
    if (grid_size_)
      ROS_ASSERT_MSG(*grid_size_==tmap_[v].x_size &&
                     *grid_size_==tmap_[v].y_size,
                     "Assumption that all grids are identical squares failed.");
    else
      grid_size_ = tmap_[v].x_size;
    const unsigned g = tmap_[v].id;

    // It's fine that this check might create entries in the maps
    if (grid_update_times_[g] > roadmap_update_times_[g])
    {
      ROS_DEBUG_STREAM_NAMED ("graph_cb", "Updating grid " << g);
      roadmap_update_times_[g] = ros::Time::now();
      
      // Load occupancy grid from db
      nm::OccupancyGrid::ConstPtr grid = getGrid(g);
      if (!grid)
      {
        ROS_WARN ("Skipping grid %u as it's not in the db yet", g);
        continue;
      }
      ROS_DEBUG_STREAM_NAMED ("graph_cb", "Loaded grid");

      {

        // When laying down waypoints, we'll use a conservatively inflated
        // grid to encourage waypoints not too close to walls
      nm::OccupancyGrid::ConstPtr inflated =
        gu::inflateObstacles(*grid, 2*robot_radius_);
      ROS_DEBUG_STREAM_NAMED ("graph_cb", "Inflated obstacles");  
      
      const PointVec waypoints = nodesOnGrid(g, roadmap_, tmap_).second;

      // For each potential waypoint on this grid
      BOOST_FOREACH (const gm::Point w, potentialWaypoints()) 
      {
        
        // Check if there's already a waypoint near that location
        bool covered = false;
        BOOST_FOREACH (const gm::Point p, waypoints) 
        {
          if (gmu::euclideanDistance(w,p)<waypoint_spacing_/2.0)
          {
            covered=true;
            break;
          }
        }
        if (covered) continue;

        // If not, try adding a waypoint near the location
        optional<gm::Point> pt = freePointNear(*inflated, w.x, w.y,
                                               waypoint_spacing_/4.0);
        if (pt)
        {
          msg::RoadmapNode info;
          info.grid = g;
          info.position = *pt;
          const unsigned id = roadmap_.addNode(info);
          ROS_DEBUG_STREAM_NAMED ("update", "Added node " << id <<
                                  " on grid " << g << " at " << toString(*pt));
        }
        else
        {
          ROS_DEBUG_NAMED ("update", "Couldn't find free cell near (%.2f, %.2f)"
                           " so not adding roadmap node", w.x, w.y);
        }
      }
      }

      // We'll use a different inflated grid for edges, that is more
      // optimistic, and allows moving through unknown space
      nm::OccupancyGrid::ConstPtr inflated2 =
        gu::inflateObstacles(*grid, robot_radius_, true);
      inflated_grid_pub_.publish(inflated2);
      

      // Add edges
      const pair<vector<unsigned>, PointVec> updated_waypoints =
        nodesOnGrid(g, roadmap_, tmap_);
      for (unsigned i=0; i<updated_waypoints.first.size(); i++)
      {
        const unsigned n=updated_waypoints.first[i];
        const gm::Point pos = positionOnGrid(n, g, roadmap_, tmap_);
        const gu::Cell c = gu::pointCell(grid->info, pos);
        const double max_dist = 2.0*waypoint_spacing_/grid->info.resolution;
        gu::TerminationCondition term(max_dist);
        ROS_DEBUG_STREAM_NAMED ("shortest_path", "Looking for paths from "
                                << n << " with max length " <<
                                2*waypoint_spacing_);
        gu::ResultPtr res = gu::singleSourceShortestPaths(*inflated2, c,
                                                          term);
        for (unsigned j=0; j<updated_waypoints.first.size(); j++)
        {
          const unsigned n2=updated_waypoints.first[j];
          if (n<=n2)
            continue;
          const GraphVertex v = roadmap_.node(n);
          const GraphVertex v2 = roadmap_.node(n2);
          const gm::Point pos = positionOnGrid(n2, g, roadmap_, tmap_);
          
          const gu::Cell c2 = gu::pointCell(grid->info, pos);
          optional<double> d = gu::distance(res, c2);
          if (d)
          {
            msg::RoadmapEdge info;
            info.id = 0; // Autogenerate id
            info.grid = g;
            info.src = n;
            info.dest = n2;
            info.cost = *d*grid->info.resolution;
            ROS_DEBUG_STREAM_NAMED ("shortest_path", "Path of length " <<
                                    info.cost << " from " << n << " to " <<
                                    n2 << " on " << g);

            pair<GraphEdge, bool> res = edge(v, v2, roadmap_);
            if (res.second) {
              msg::RoadmapEdge& existing = roadmap_[res.first];
              ROS_ASSERT_MSG(existing.src == info.src &&
                             existing.dest == info.dest,
                             "Edge (%u, %u) didn't match (%u, %u)",
                             existing.src, existing.dest, info.src, info.dest);
              existing.cost = info.cost;
              ROS_DEBUG_STREAM_NAMED ("update", "Updating edge cost from "
                                      << n << " to " << n2 << " to " <<
                                      info.cost);
            }
            else
              roadmap_.addEdge(info);
          }
        }
      }
    }
  }

  // For now, we visualize and publish roadmap in the graph callback
  visualize(roadmap_, vis_pub_, tf_, vis_frame_, visualize_node_ids_, path_);
  roadmap_pub_.publish(toRosMessage(roadmap_));
  
  ROS_DEBUG_STREAM_NAMED ("graph_cb", "Leaving graph callback");
}


/************************************************************
 * Localization update
 ***********************************************************/

void RoadmapNode::locCB (const gm::PoseStamped& l)
{
  ROS_DEBUG_STREAM_NAMED ("loc_cb", "Entering localization callback");
  Lock lock(mutex_);
  if (!last_loc_ || last_loc_->header.frame_id!=l.header.frame_id)
    ROS_DEBUG_STREAM_NAMED ("loc_cb", "New local frame " << l.header.frame_id);
  last_loc_ = l;
  ROS_DEBUG_STREAM_NAMED ("loc_cb", "Leaving localization callback");
}


/************************************************************
 * Grid update
 ***********************************************************/

void RoadmapNode::gridUpdateCB (const std_msgs::String& m)
{
  mr::Metadata metadata(m.data);
  const unsigned g = metadata.getIntField("id");

  // When the notification isn't about a grid update, don't do anything
  if (g==0)
    return;

  Lock l(mutex_);
  grid_update_times_[g] = ros::Time::now();
}

void RoadmapNode::pathCB (const msg::RoadmapPath& m)
{
  Lock l(mutex_);
  path_ = m.path;
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "roadmap_builder");
  topological_roadmap::RoadmapNode node;
  ros::spin();
  return 0;
}
