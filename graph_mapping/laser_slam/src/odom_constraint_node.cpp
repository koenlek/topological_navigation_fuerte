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
 * Node that publishes constraints based on an odometry frame looked up in tf
 *
 * \author Bhaskara Marthi
 */

#include <pose_graph/diff_subscriber.h>
#include <graph_mapping_utils/utils.h>
#include <graph_mapping_msgs/GraphConstraint.h>
#include <graph_mapping_msgs/ConstraintGraphDiff.h>
#include <ros/ros.h>
#include <boost/optional.hpp>

namespace laser_slam {

namespace pg = pose_graph;
namespace msg = graph_mapping_msgs;
namespace util = graph_mapping_utils;

using std::string;
using boost::optional;

typedef optional<const msg::ConstraintGraphDiff&> OptionalDiff;

/************************************************************
 * Class
 ***********************************************************/

class OdomConstraints {
public:
	OdomConstraints(ros::NodeHandle& param_nh);

private:

	tf::Pose fixedFramePose(const ros::Time& t);
	void diffCB(OptionalDiff diff, const pg::ConstraintGraph& g);

	/****************************************
	 * Params
	 ****************************************/

	ros::NodeHandle param_nh_;
	const string base_frame_;
	const string odom_frame_;

	/****************************************
	 * State
	 ****************************************/

	optional<unsigned> last_node_;
	optional<tf::Pose> last_odom_pose_;

	/****************************************
	 * Objects
	 ****************************************/

	ros::NodeHandle nh_;
	tf::TransformListener tf_;
	pg::DiffSubscriber diff_sub_;
	ros::Publisher constraint_pub_;

};

/************************************************************
 * Init
 ***********************************************************/

OdomConstraints::OdomConstraints(ros::NodeHandle& param_nh) :
		param_nh_(param_nh), base_frame_(
				util::getParam<string>(param_nh_, "base_frame")), odom_frame_(
				util::getParam<string>(param_nh_, "fixed_frame")), diff_sub_(
				boost::bind(&OdomConstraints::diffCB, this, _1, _2)), constraint_pub_(
				nh_.advertise<msg::GraphConstraint>("graph_constraints", 10)) {
	ROS_INFO("odom_constraint_node initialized");
}

/************************************************************
 * Constraints
 ***********************************************************/

tf::Pose OdomConstraints::fixedFramePose(const ros::Time& t) {
	tf_.waitForTransform(odom_frame_, base_frame_, t, ros::Duration(1.0));
	tf::StampedTransform tr;
	tf_.lookupTransform(odom_frame_, base_frame_, t, tr);
	return tr;
}

void OdomConstraints::diffCB(OptionalDiff diff, const pg::ConstraintGraph& g) {
	if (!diff) {
		ROS_INFO(
				"Not adding constraint since got full graph rather than a diff");
	} else if (diff->new_nodes.size() != 1) {
		ROS_INFO("Not adding constraint since diff had %zu new nodes",
				diff->new_nodes.size());
	} else {
		const unsigned n = diff->new_nodes[0].id;
		try {
			const tf::Pose odom_pose = fixedFramePose(
					diff->new_node_timestamps[0]);
			if (last_node_) {
				msg::GraphConstraint constraint;
				constraint.src = *last_node_;
				constraint.dest = n;
				constraint.constraint.pose = util::toPose(
						util::relativePose(odom_pose, *last_odom_pose_));
				constraint.constraint.precision = util::makePrecisionMatrix(1,
						1, 1, 1);
				constraint_pub_.publish(constraint);
				/*   ROS_INFO_STREAM ("Added constraint to " << *last_node_ <<
				 " given new node " << n <<
				 " at " << util::toString(odom_pose));*/
			} else {
				ROS_INFO("First node %u seen ", n);
			}

			last_node_ = n;
			last_odom_pose_ = odom_pose;
			/* ROS_INFO_STREAM ("Last node is now " << *last_node_ << " at "
			                << util::toString(*last_odom_pose_));*/
		} catch (tf::TransformException& e) {
			ROS_WARN("Ignoring new node %u due to tf exception '%s'", n,
					e.what());
		}
	}
}

} // namespace

int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_constraint_node");
	ros::NodeHandle param_nh("laser_slam_node");
	laser_slam::OdomConstraints node(param_nh);
	ros::spin();
	return 0;
}
