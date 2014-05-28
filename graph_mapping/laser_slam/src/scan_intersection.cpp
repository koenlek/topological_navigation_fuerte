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
 * Implementation of scan_intersection.h
 *
 * \author Bhaskara Marthi
 */

#include <laser_slam/scan_intersection.h>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <ros/assert.h>

namespace laser_slam
{

using std::vector;
using boost::optional;
typedef vector<tf::Transform> Scans;

inline
double pointToLineSegment (const tf::Vector3& p, const tf::Vector3& p1,
                           const tf::Vector3& p2)
{
  const double dx = p.x() - p1.x();
  const double dy = p.y() - p1.y();
  const double dx1 = p2.x() - p1.x();
  const double dy1 = p2.y() - p1.y();

  return fabs(dx*dy1-dy*dx1)/sqrt(dx1*dx1+dy1*dy1);
}


// We compute a Monte Carlo estimate of the proportion of the new
// scan's polygon that isn't covered by other scans.
// This is done by looking at each grid point in the scan polygon
// and checking if it belongs to any of the other scans.
double ScanIntersection::unseenProportion (const tf::Transform& scan,
                                           const Scans& scans) const
{
  unsigned unseen=0;
  BOOST_FOREACH (const tf::Vector3& pt, grid_) {
    const tf::Vector3 p = scan*pt;
    bool found=false;
    BOOST_FOREACH (const tf::Transform& ref, scans) {
      if (contains(ref, p)) {
        found=true;
        break;
      }
    }
    if (!found)
      unseen++;
  }
  return (double)unseen/(double)grid_.size();
}


ScanIntersection::ScanIntersection (const ScanIntersection& i) :
  poly_(i.poly_), grid_(i.grid_), center_(i.center_),
  inradius2_(i.inradius2_), circumradius2_(i.circumradius2_)
{


}

ScanIntersection::ScanIntersection (vector<tf::Vector3> poly,
                                    const unsigned grid_size) :
  poly_(poly)
{

  ROS_ASSERT_MSG (poly[0].distance(tf::Vector3(0,0,0))<1e-6,
                  "First point of polygon %.2f, %.2f, %.2f wasn't origin",
                  poly[0].x(), poly[0].y(), poly[0].z());
  
  // Compute centroid and axis-aligned bounding box of polygon
  using std::min;
  using std::max;
  double sx=0, sy=0;
  optional<double> min_x, min_y, max_x, max_y;
  BOOST_FOREACH (const tf::Vector3& p, poly) {
    sx += p.x();
    sy += p.y();
    if (!min_x || p.x()<*min_x)
      min_x = p.x();
    if (!max_x || p.x()>*max_x)
      max_x = p.x();
    if (!min_y || p.y()<*min_y)
      min_y = p.y();
    if (!max_y || p.y()>*max_y)
      max_y = p.y();
  }
  center_.setX(sx/poly.size());
  center_.setY(sy/poly.size());

  // Compute inradius, circumradius
  boost::optional<double> inradius, circumradius;
  
  for (int i=0; i<(int)poly.size(); i++) {
    int j = i==0 ? poly.size()-1 : i-1;
    const double ri = pointToLineSegment(center_, poly[j], poly[i]);
    if (!inradius || ri<*inradius)
      inradius = ri;
    const double rc = center_.distance(poly[i]);
    if (!circumradius || rc > *circumradius)
      circumradius = rc;
    ROS_ASSERT_MSG (ri < rc, "Polygon is not convex!");
  }
  inradius2_ = (*inradius) * (*inradius);
  circumradius2_ = (*circumradius) * (*circumradius);
  

  // Lay down a grid over the interior of the polygon
  const double x_step = (*max_x - *min_x)/sqrt(grid_size);
  const double y_step = (*max_y - *min_y)/sqrt(grid_size);
  
  tf::Transform id;
  id.setIdentity();
  for (double x=*min_x; x<*max_x; x+=x_step)
  {
    for (double y=*min_y; y<*max_y; y+=y_step)
    {
      const tf::Vector3 p(x,y,0);
      if (contains(id, p))
        grid_.push_back(tf::Vector3(x,y,0));
    }
  }

}


  

} // namespace
