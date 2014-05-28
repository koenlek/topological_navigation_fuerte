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
 * Code for telling when a laser scan covers a lot of different space than
 * a bunch of existing ones
 *
 * \author Bhaskara Marthi
 */

#ifndef LASER_SLAM_SCAN_INTERSECTION_H
#define LASER_SLAM_SCAN_INTERSECTION_H

#include <vector>
#include <tf/LinearMath/Transform.h>

namespace laser_slam
{

/// \brief Construct with the dimensions of the scan polygon, and then
/// repeatedly call unseenProportion
class ScanIntersection
{
public:
  /// \brief Constructor precomputes some things given the polygon
  /// \param grid_size The approximate number of points in the grid
  /// used for computing the Monte Carlo estimate of the intersection
  /// area
  ScanIntersection (std::vector<tf::Vector3> poly,
                    unsigned grid_size=100);

  ScanIntersection (const ScanIntersection& i);

  /// \retval Number between 0 and 1.  1 means new scan is totally
  /// nonintersecting with existing ones and 0 means it's totally
  /// covered.
  double unseenProportion (const tf::Transform& scan,
                           const std::vector<tf::Transform>& ref_scans) const;

  /// \brief Does scan polygon given scan pose \a scan contain point \a pt?
  inline
  bool contains (const tf::Transform& scan, const tf::Vector3& pt) const
  {
    // Transform p into relative coordinates in frame of \a scan
    const tf::Vector3 p = scan.inverse()*pt;

    // Short-circuit test if point is too close or too far from center
    const double dist = p.distance2(center_);
    if (dist > circumradius2_)
      return false;
    else if (dist < inradius2_)
      return true;

    // Point-in-polygon test
    int i, j = 0;
    bool c = false;
    for (i=0, j=poly_.size()-1; i<(int)poly_.size(); j = i++) {
      if ( ((poly_[i].y()>p.y()) != (poly_[j].y()>p.y())) &&
           (p.x() < (poly_[j].x()-poly_[i].x()) * (p.y()-poly_[i].y()) /
            (poly_[j].y()-poly_[i].y()) + poly_[i].x()) )
        c = !c;
    }
    return c;
  }


private:


  std::vector<tf::Vector3> poly_;
  std::vector<tf::Vector3> grid_;
  tf::Vector3 center_;
  float inradius2_, circumradius2_;
};
  
  

  





} // namespace

#endif // include guard
