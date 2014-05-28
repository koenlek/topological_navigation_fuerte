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
 * Tests for topological_roadmap
 *
 * \author Bhaskara Marthi
 */

#include <topological_roadmap/shortest_paths.h>
#include <topological_roadmap/ros_conversion.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>

namespace rm=topological_roadmap;
namespace msg=topological_nav_msgs;

using boost::assign::operator+=;
using std::ostream;
using std::vector;
using boost::optional;


template <class T>
ostream& operator<< (ostream& str, const vector<T>& s)
{
  std::copy(s.begin(), s.end(), std::ostream_iterator<T>(str, " "));
  return str;
}

unsigned addNode (rm::Roadmap* r, const unsigned grid)
{
  msg::RoadmapNode info;
  info.id = 0;
  info.grid = grid;
  info.position.x = 42;
  info.position.y = -42;
  return r->addNode(info);
}

unsigned addEdge (rm::Roadmap* r, const unsigned n1, const unsigned n2,
                  const unsigned grid, const double cost)
{
  msg::RoadmapEdge info;
  info.id = 0;
  info.src = n1;
  info.dest = n2;
  info.grid = grid;
  info.cost = cost;
  return r->addEdge(info);
}


TEST(topological_roadmap, RoadmapTest)
{
  rm::Roadmap r;
  const unsigned n1 = addNode(&r, 1);
  const unsigned n2 = addNode(&r, 1);
  const unsigned n3 = addNode(&r, 2);
  const unsigned n4 = addNode(&r, 2);
  const unsigned n5 = addNode(&r, 3);
  const unsigned n6 = addNode(&r, 2);
  addEdge(&r, n1, n2, 1, 5);
  addEdge(&r, n3, n1, 1, 1);
  addEdge(&r, n2, n3, 1, 2);
  addEdge(&r, n4, n2, 2, 10);
  addEdge(&r, n5, n6, 2, 1);
  rm::NodeVec p;
  p += n1, n3, n2, n4;

  {
    // Check shortest path
    rm::ResultPtr sp = shortestPaths(r, n1);
    optional<rm::Path> p1 = extractPath(sp, n4);
    ASSERT_TRUE(p1);
    EXPECT_EQ(p, p1->first);

    // Check infeasible path
    optional<rm::Path> p2 = extractPath(sp, n6);
    ASSERT_TRUE(!p2);
  }

  // Test ros conversion
  msg::TopologicalRoadmap m = *rm::toRosMessage(r);
  rm::Roadmap r2 = rm::fromRosMessage(m);

  {
    // Check shortest path is still same
    rm::ResultPtr sp = shortestPaths(r2, n1);
    optional<rm::Path> p1 = extractPath(sp, n4);
    ASSERT_TRUE(p1);
    EXPECT_EQ(p, p1->first);

    // Check infeasible path
    optional<rm::Path> p2 = extractPath(sp, n6);
    ASSERT_TRUE(!p2);
  }  
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "topological_roadmap_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
