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
 * Conversion of various types to string / operator<<
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_MAPPING_UTILS_TO_STRING_H
#define GRAPH_MAPPING_UTILS_TO_STRING_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <set>
#include <numeric>
#include <tf/transform_datatypes.h>

namespace graph_mapping_utils
{

namespace gm=geometry_msgs;

using std::string;
using boost::format;

string toString (const gm::Pose2D& pose);

string toString (const gm::Point& p);

string toString2D (const gm::Pose& pose);
string toString2D (const tf::Pose& pose);

string toString (const gm::Pose& pose);


template <class T>
std::string concatenate (const std::string& s, const T& x)
{
  return s + std::string(" ") + boost::lexical_cast<std::string>(x);
}


template <class T>
std::string toString (const std::set<T>& set)
{
  const std::string s = std::accumulate(set.begin(), set.end(), std::string(""), concatenate<T>);
  return std::string("[") + s + std::string("]");
}

template <class T>
std::string toString (const std::vector<T>& v)
{
  const std::string s = std::accumulate(v.begin(), v.end(), std::string(""), concatenate<T>);
  return std::string("[") + s + std::string("]");
}

std::string toString (const tf::Transform& t);
std::string toString (const tf::Vector3& v);
std::string toString (const tf::Quaternion& q);

} // namespace

#endif // include guard
