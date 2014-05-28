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
 * Functions for simple 2d exploration
 *
 * \author Bhaskara Marthi
 */

#ifndef EXPLORE_2D_EXPLORATION_H
#define EXPLORE_2D_EXPLORATION_H

#include <nav_msgs/OccupancyGrid.h>

namespace explore_2d
{

/// \brief Holds the parameters to the Explorer.  Pass this to the constructor of Explorer.
struct ExplorerParams
{
  ExplorerParams (double radius) :
    robot_radius(radius)
  {}

  double robot_radius;
};


/// \brief Holds the state of the exploration
class Explorer 
{
public:

  /// \brief Constructor.
  Explorer(const ExplorerParams& params);

  /// \brief Set the current occupancy grid
  void updateOccupancyGrid (const nav_msgs::OccupancyGrid& grid);

  /// \brief Get the next nav goal to pursue
  geometry_msgs::Pose nextNavGoal ();

  
private:

  const ExplorerParams params_;

  nav_msgs::OccupancyGrid::ConstPtr inflated_grid_;

};

} // namespace

#endif // include guard
