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
 * Implementation of Exploration.h
 *
 * \author Bhaskara Marthi
 */

#include <explore_2d/exploration.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <cstdlib>
#include <ctime>

namespace explore_2d
{

namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;
namespace gm=geometry_msgs;


Explorer::Explorer (const ExplorerParams& params) :
  params_(params)
{
  srand(time(NULL));
}


void Explorer::updateOccupancyGrid (const nm::OccupancyGrid& grid)
{
  inflated_grid_ = gu::inflateObstacles(grid, params_.robot_radius);
}


gm::Pose Explorer::nextNavGoal ()
{
  ROS_ASSERT_MSG (inflated_grid_, "Can't get a nav goal until the grid is set");
  const nm::MapMetaData& info = inflated_grid_->info;
  const unsigned NUM_TRIES_UNKNOWN_SPACE=5;

  gu::Cell c;
  unsigned n=0;

  // Outer loop is so we first look for the border between known and unknown space,
  // and then between known and occupied space  
  while (true) {

    n++;

    // choose random points until we find one that is free
    const unsigned num_cells = info.width*info.height;
    unsigned i = -1;
    do {
      i = rand() % num_cells;
    } while (inflated_grid_->data[i] != gu::UNOCCUPIED);
    c = gu::indexCell(info, i);

    // Next, move towards edge from that cell until we reach the edge of free space
    const int dx = c.x > (int) info.width/2 ? 1 : -1;
    const int dy = c.y > (int) info.height/2 ? 1 : -1;

    gu::Cell c2;
    while (true) {
      c2 = gu::Cell(c.x+dx, c.y+dy);
      if (!gu::withinBounds(info, c2))
        break;
      if (inflated_grid_->data[cellIndex(info, c2)] != gu::UNOCCUPIED)
        break;
      c = c2;
    }
    if (!gu::withinBounds(info, c2) || inflated_grid_->data[cellIndex(info, c2)] == gu::UNKNOWN ||
        n > NUM_TRIES_UNKNOWN_SPACE)
      break;
  }

  
  gm::Pose p;
  p.position = gu::cellCenter(info, c);
  p.orientation.w = 1.0;

  return p;
}


} // namespace
