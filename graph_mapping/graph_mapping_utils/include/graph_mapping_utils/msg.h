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
 * Utilities for creating and manipulating graph mapping messages
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_MAPPING_UTILS_MSG_H
#define GRAPH_MAPPING_UTILS_MSG_H

#include <graph_mapping_msgs/Node.h>
#include <graph_mapping_msgs/Edge.h>
#include <graph_mapping_msgs/NodePoses.h>
#include "geometry.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>


namespace graph_mapping_utils
{

using graph_mapping_msgs::Node;
using graph_mapping_msgs::Edge;
using graph_mapping_msgs::PoseWithPrecision;
using graph_mapping_msgs::NodePoses;
using std::string;

inline
Node makeNode (unsigned n)
{
  Node node;
  node.id = n;
  return node;
}

inline
Edge makeEdge (unsigned e, unsigned from, unsigned to, const PoseWithPrecision& constraint)
{
  Edge edge;
  edge.id = e;
  edge.constraint.src = from;
  edge.constraint.dest = to;
  edge.constraint.constraint = constraint;
  return edge;
}

inline
unsigned index(const unsigned i, const unsigned j)
{
  return 6*i + j;
}

/// Create a precision matrix given the precisions of x, y, and theta, and the joint precision of x-y
inline
boost::array<double, 36> makePrecisionMatrix (const double x, const double y, const double xy, const double theta)
{
  boost::array<double, 36> prec;
  std::fill(prec.begin(), prec.end(), 0.0);
  prec[index(0,0)] = x;
  prec[index(1,1)] = y;
  prec[index(0,1)] = xy;
  prec[index(1,0)] = xy;
  prec[index(5,5)] = theta;

  return prec;
}


typedef std::map<unsigned, tf::Pose> NodePoseMap;
NodePoseMap fromMsg (const NodePoses& poses);
NodePoses::ConstPtr toMsg (const NodePoseMap& poses);

struct InvalidGraphLocalizationException: public std::logic_error
{
  InvalidGraphLocalizationException (const string& frame) :
    std::logic_error((boost::format("Frame %1% is not of form nodeXXX")
                      % frame).str())
  {}
};

/// \param A localization wrt the graph, which is represented as a pose stamped
/// where the frame_id looks like 'node24'
/// \retval Extracted node id corresponding to the frame
/// \throw InvalidGraphLocalizationException if frame_id isn't of the above form
inline
unsigned refNode (const geometry_msgs::PoseStamped& p)
{
  try
  {
    return boost::lexical_cast<unsigned>((p.header.frame_id.c_str()+4));
  }
  catch (boost::bad_lexical_cast&)
  {
    throw InvalidGraphLocalizationException(p.header.frame_id);
  }
}

inline
string nodeFrame (const unsigned n)
{
  return ("node" + boost::lexical_cast<string>(n));
}

/// Get the optimized current pose given localization and optimized graph poses
inline
tf::Pose optimizedPose (const gm::PoseStamped& loc,
                        const std::map<unsigned, tf::Pose>& opt_poses)
{
  const tf::Pose& ref_pose = keyValue(opt_poses, refNode(loc));
  return toPose(transformPose(ref_pose, loc.pose));
}


} // namespace

#endif // include guard


