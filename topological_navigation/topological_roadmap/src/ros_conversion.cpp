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
 * Implementation of ros_conversion.h
 *
 * \author Bhaskara Marthi
 */


#include <topological_roadmap/ros_conversion.h>
#include <graph_mapping_utils/to_string.h>
#include <visualization_msgs/Marker.h>
#include <tf/exceptions.h>
#include <boost/foreach.hpp>

namespace topological_roadmap
{
namespace vm=visualization_msgs;
namespace gm=geometry_msgs;
namespace msg=topological_nav_msgs;
namespace gmu=graph_mapping_utils;

typedef std::vector<unsigned> Path;

msg::TopologicalRoadmap::Ptr toRosMessage (const Roadmap& r)
{
  msg::TopologicalRoadmap::Ptr m =
    boost::make_shared<msg::TopologicalRoadmap>();
  BOOST_FOREACH (const GraphVertex v, vertices(r)) 
    m->nodes.push_back(r[v]);
  BOOST_FOREACH (const GraphEdge e, edges(r)) 
    m->edges.push_back(r[e]);

  return m;
}


Roadmap fromRosMessage (const msg::TopologicalRoadmap& m)
{
  Roadmap r;

  BOOST_FOREACH (const msg::RoadmapNode& info, m.nodes) 
    r.addNode(info);
  BOOST_FOREACH (const msg::RoadmapEdge& info, m.edges) 
    r.addEdge(info);
  r.recomputeIndexes();

  return r;
}


gm::Point nodePos (const Roadmap& r, tf::TransformListener& tf,
                   const std::string& frame, const unsigned n)
{
  gm::PointStamped in, out;
  const msg::RoadmapNode& info = r.nodeInfo(n);
  in.point = info.position;
  in.header.stamp = ros::Time();
  in.header.frame_id = topological_map_2d::gridFrame(info.grid);
  tf.transformPoint(frame, in, out);
  return out.point;
}

void visualizeNodes (const Roadmap& r, ros::Publisher& pub,
                     tf::TransformListener& tf, const std::string& frame,
                     const bool use_node_ids)
{
  vm::Marker nodes;
  nodes.header.frame_id = frame;
  nodes.header.stamp = ros::Time::now();
  nodes.id = 1;
  nodes.ns = "roadmap";
  nodes.action = vm::Marker::ADD;
  nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.1;
  nodes.color.r = 1.0;
  nodes.color.g = 1.0;
  nodes.color.b = 0.0;
  nodes.color.a = 1.0;
  nodes.pose.orientation.w = 1.0;

  if (use_node_ids)
    nodes.type = vm::Marker::SPHERE;
  else
    nodes.type = vm::Marker::SPHERE_LIST;
  
  BOOST_FOREACH (const GraphVertex& v, vertices(r)) 
  {
    try
    {
      if (use_node_ids)
      {
        nodes.id = r[v].id;
        nodes.pose.position = nodePos(r, tf, frame, r[v].id);
        pub.publish(nodes);
      }
      else
      {
        nodes.points.push_back(nodePos(r, tf, frame, r[v].id));
      }
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN_STREAM_THROTTLE (1.0, "TF exception " << e.what() <<
                                " when visualizing roadmap");
    }
  }

  if (!use_node_ids)
    pub.publish(nodes);
}

void visualizeEdges (const Roadmap& r, ros::Publisher& pub,
                     tf::TransformListener& tf, const std::string& frame,
                     const Path& plan)
{
  vm::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "roadmap";
  m.type = vm::Marker::LINE_LIST;
  m.action = vm::Marker::ADD;
  m.scale.x = 0.05;
  m.color.r = 0.15;
  m.color.g = 0.55;
  m.color.b = 0.15;
  m.color.a = 1.0;

  vm::Marker path = m;
  path.id = 99999; // Avoid conflicting with node ids
  path.color.r = 0.75;
  path.color.g = 0.2;
  path.scale.x = 0.07;

  BOOST_FOREACH (const GraphEdge& e, boost::edges(r)) 
  {
    try
    {
      const unsigned src_node = r[e].src;
      const unsigned dest_node = r[e].dest;
      const gm::Point src = nodePos(r, tf, frame, src_node);
      const gm::Point dest = nodePos(r, tf, frame, dest_node);
      bool on_path = false;
      for (unsigned i=0; i+1<plan.size(); i++)
      {
        if ((plan[i] == r[e].src && plan[i+1] == r[e].dest) ||
            (plan[i+1] == r[e].src && plan[i] == r[e].dest))
          on_path=true;
      }
      if (on_path) 
      {
        path.points.push_back(src);
        path.points.push_back(dest);
      }
      else
      {
        m.points.push_back(src);
        m.points.push_back(dest);
      }
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN_STREAM_THROTTLE (1.0, "TF exception " << e.what() <<
                                " when visualizing roadmap");
    }
  }

  pub.publish(m);
  pub.publish(path);
}



void visualize (const Roadmap& r, ros::Publisher& pub,
                tf::TransformListener& tf, const std::string& frame,
                const bool use_node_ids, const Path& path)
{
  visualizeNodes(r, pub, tf, frame, use_node_ids);
  visualizeEdges(r, pub, tf, frame, path);
}


} // namespace
