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

#include <laser_slam/odom_localizer.h>
#include <graph_mapping_utils/geometry.h>
#include <graph_mapping_utils/to_string.h>

namespace laser_slam
{

namespace gs=graph_slam;
namespace gm=geometry_msgs;
namespace pg=pose_graph;
namespace util=graph_mapping_utils;

OdomLocalizer::OdomLocalizer ()
{
  ROS_INFO ("Initializing OdomLocalizer");
}

gs::LocalizationDistribution::Ptr OdomLocalizer::update (const Graph&, 
                                                         gs::LocalizationDistribution::ConstPtr localization,
                                                         const tf::Pose& rel_pose, const ScanPtr& scan)
{
  gs::LocalizationDistribution::Ptr dist(new gs::LocalizationDistribution());
  dist->stamp = scan->header.stamp;
  dist->samples.resize(1);
  ROS_ASSERT(localization->samples.size()==1);

  dist->samples[0].node = localization->samples[0].node;
  dist->samples[0].offset = util::absolutePose(localization->samples[0].offset, rel_pose);

  ROS_DEBUG_STREAM_COND_NAMED (fabs(rel_pose.getOrigin().x()) > 1e-3,
                               "localization", "In odom localization update with relative pose " <<
                               util::toString(rel_pose) << "; old loc was " << localization->samples[0] <<
                               " and new is " << dist->samples[0]);
  return dist;
}


} // namespace laser_slam
