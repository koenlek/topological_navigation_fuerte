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
 */

#include <gtest/gtest.h>
#include <ros/assert.h>
#include <laser_slam/scan_intersection.h>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>

using std::vector;
namespace ls=laser_slam;

const double PI=3.14159265;

tf::Transform makePose (const double x, const double y, const double th)
{
  return tf::Transform(tf::Quaternion(tf::Vector3(0,0,1), th),
                      tf::Vector3(x,y,0));
}



TEST(LaserSlam, ScanIntersection)
{
  vector<tf::Vector3> poly(4);
  poly[1].setX(-5);
  poly[1].setY(1);
  poly[2].setY(8);
  poly[3].setX(5);
  poly[3].setY(1);
  
  ls::ScanIntersection scan_int(poly);

  const tf::Vector3 p(5, 5, 0);
  const tf::Transform p1 = makePose(0,0,0);
  const tf::Transform p2 = makePose(0,0,PI);
  const tf::Transform p3 = makePose(4,4,0);
  const tf::Transform p4 = makePose(4,4,PI);
  const tf::Transform p5 = makePose(-0.1, -0.1, 0);
  
  EXPECT_TRUE (!scan_int.contains(p1, p));
  EXPECT_TRUE (!scan_int.contains(p2, p));
  EXPECT_TRUE (scan_int.contains(p3, p));
  EXPECT_TRUE (!scan_int.contains(p4, p));

  vector<tf::Transform> r1;
  r1.push_back(p1);
  vector<tf::Transform> r2;
  r2.push_back(p2);

  EXPECT_NEAR (scan_int.unseenProportion(p5, r1), 0, 0.1);
  EXPECT_NEAR (scan_int.unseenProportion(p5, r2), 1, 0.1);
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

