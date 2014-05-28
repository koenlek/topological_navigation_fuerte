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
 * Implementation of geometry.h
 *
 * \author Bhaskara Marthi
 */

#include <graph_mapping_utils/geometry.h>
#include <boost/foreach.hpp>
#include <stdexcept>

namespace graph_mapping_utils
{

namespace sm=sensor_msgs;

gm::Pose2D projectToPose2D (const tf::Transform& pose)
{
  gm::Pose2D pose2d;
  pose2d.x = pose.getOrigin().x();
  pose2d.y = pose.getOrigin().y();
  pose2d.theta = tf::getYaw(pose.getRotation());
  return pose2d;
}

gm::Pose2D projectToPose2D (const gm::Pose& pose)
{
  gm::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf::getYaw(pose.orientation);
  return pose2d;
}


/// Construct a pose2d
gm::Pose2D makePose2D (double x, double y, double theta)
{
  gm::Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  return p;
}

tf::Pose makePose (double x, double y, double theta)
{
  return tf::Pose(tf::createQuaternionFromYaw(theta), tf::Vector3(x, y, 0));
}


gm::Point barycenter (const sensor_msgs::PointCloud& cloud)
{
  double sx=0, sy=0, sz=0;
  if (cloud.points.empty())
    throw std::invalid_argument("Can't find barycenter of empty cloud");
  BOOST_FOREACH (const gm::Point32& p, cloud.points)
  {
    sx += p.x;
    sy += p.y;
    sz += p.z;
  }
  gm::Point b;
  const unsigned n = cloud.points.size();
  b.x = sx/n;
  b.y = sy/n;
  b.z = sz/n;
  return b;
}

tf::Vector3 barycenter (const sm::LaserScan& scan)
{
  unsigned n = 0;
  double theta = scan.angle_min;
  double sx=0, sy=0;
  BOOST_FOREACH (const double r, scan.ranges)
  {
    if ((scan.range_min < r) && (r < scan.range_max))
    {
      sx += r*cos(theta);
      sy += r*sin(theta);
      n++;
    }
    theta += scan.angle_increment;
  }
  ROS_ASSERT_MSG (n>0, "Can't find barycenter of scan because all "
                  "%zu points are out of range", scan.ranges.size());
  return tf::Vector3(sx/n, sy/n, 0);  
}

tf::Pose toPose (const gm::Pose2D& p)
{
  return makePose(p.x, p.y, p.theta);
}


/// Return dimensions of a map centered at pose of the given size (meters) and resolution
/*nm::MapMetaData getMapInfo (const tf::Pose& pose, const double size, const double resolution)
{

  gm::Pose local_origin;
  local_origin.position.x = -size/2;
  local_origin.position.y = -size/2;
  local_origin.orientation.w = 1.0;

  nm::MapMetaData info;
  info.resolution = resolution;
  info.width = size/resolution;
  info.height = size/resolution;
  info.origin = transformPose(pose, local_origin);

  return info;
}
*/
/*nm::OccupancyGrid transformGrid (const tf::Transform& trans, const nm::OccupancyGrid& g)
{
  nm::OccupancyGrid g2(g);
  g2.info.origin = transformPose(trans, g.info.origin);
  return g2;
  }*/

} // namespace
