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
 * Operations on and conversions between the ros and tf/bullet geometry types
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_MAPPING_UTILS_GEOMETRY_H
#define GRAPH_MAPPING_UTILS_GEOMETRY_H

#include "general.h"
#include <graph_mapping_msgs/LocalizationDistribution.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

namespace graph_mapping_utils
{

namespace gm=geometry_msgs;


/// \a pose1 and \a pose2 are poses specified in some reference frame.
/// This function returns the representation of \a pose1 in the frame at \a pose2.
inline
tf::Pose relativePose (const tf::Pose& pose1, const tf::Pose& pose2)
{
  return pose2.inverseTimes(pose1);
}

/// Return the transform that sends pose1 to pose2
inline
tf::Transform transformBetween (const tf::Pose& pose1, const tf::Pose& pose2)
{
  return pose2*pose1.inverse();
}

/// Apply Bullet transformation to a ROS point
inline
gm::Point32 transformPoint (const tf::Transform& trans, const gm::Point32& point)
{
  tf::Vector3 p(point.x, point.y, point.z);
  tf::Vector3 p2 = trans*p;
  gm::Point32 ret;
  ret.x = p2.x();
  ret.y = p2.y();
  ret.z = p2.z();
  return ret;
}


/// Apply Bullet transformation to a ROS point
inline
gm::Point transformPoint (const tf::Transform& trans, const gm::Point& point)
{
  tf::Vector3 p;
  tf::pointMsgToTF(point, p);
  const tf::Vector3 p2 = trans*p;
  gm::Point ret;
  tf::pointTFToMsg(p2, ret);
  return ret;
}


/// Apply Bullet transformation to a ROS pose
inline
gm::Pose transformPose (const tf::Transform& trans, const gm::Pose& pose)
{
  tf::Pose bullet_pose;
  tf::poseMsgToTF(pose, bullet_pose);
  gm::Pose ret;
  tf::poseTFToMsg(trans*bullet_pose, ret);
  return ret;
}


inline
tf::Vector3 toPoint (const gm::Point& p)
{
  return tf::Vector3(p.x, p.y, p.z);
}

inline
gm::Point toPoint (const tf::Vector3& p)
{
  gm::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return pt;
}

inline
gm::Pose toPose (const tf::Pose& p)
{
  gm::Pose p2;
  tf::poseTFToMsg(p, p2);
  return p2;
}

inline
tf::Pose toPose (const gm::Pose& p)
{
  const gm::Quaternion& q(p.orientation);
  const gm::Point& v(p.position);
  return tf::Pose(tf::Quaternion(q.x, q.y, q.z, q.w),
                  tf::Vector3(v.x, v.y, v.z));
}

inline
double length (const tf::Vector3& v)
{
  return v.x()*v.x() + v.y()*v.y() + v.z()*v.z();
}

inline
double length (const gm::Point& p)
{
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


/// Project Pose to Pose2D
gm::Pose2D projectToPose2D (const gm::Pose& pose);

/// Project tf pose to pose2d
gm::Pose2D projectToPose2D (const tf::Pose& pose);

/// Construct a pose2d
gm::Pose2D makePose2D (double x, double y, double theta=0.0);

/// Construct a tf::Pose from x,y,theta
tf::Pose makePose (double x, double y, double theta);

/// Make a pose from a pose2d
tf::Pose toPose (const gm::Pose2D& p);

/// Construct a geometry_msgs Point
inline
gm::Point makePoint (const double x, const double y, const double z=0.0)
{
  gm::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}


/// Adjust a pose given a relative offset
inline
gm::Pose absolutePose (const gm::Pose& base, const tf::Pose& offset)
{
  return toPose(toPose(base)*offset);
}

/// Adjust a pose given a relative offset
inline
tf::Pose absolutePose (const tf::Pose& base, const gm::Pose& offset)
{
  return base*toPose(offset);
}


/// Return the barycenter of the cloud, in the same frame as the cloud itself
gm::Point barycenter (const sensor_msgs::PointCloud& cloud);

/// Return the barycenter of a laser scan (in the laser frame)
tf::Vector3 barycenter (const sensor_msgs::LaserScan& scan);

/// Return euclidean distance between two 3d-positions
inline
double euclideanDistance (const gm::Point& p1, const gm::Point& p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}



} // namespace

#endif // include guard
