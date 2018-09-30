/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the Replanner class, which listens for replanning requests,
// makes the appropriate service calls, and re-sends the service response
// back on the appropriate topic.
//
// NOTE! This class exists because the Planner interface is service-based,
// rather than pub/sub-based, and the PlannerManager cannot afford to wait for
// the service calls to finish (i.e. for the planner to compute) before
// sending the next planner reference state.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/planning/replanner.h>

namespace fastrack {
namespace planning {

// Callback for processing replanning requests.
void Replanner::ReplanRequestCallback(
  const fastrack_msgs::ReplanRequest::ConstPtr& msg) {
  // Make sure server is connected.
  if (!replan_srv_) {
    ROS_ERROR("%s: Replan server was disconnected.", name_.c_str());
    return;
  }

  // Make a service call.
  fastrack_srvs::Replan replan;
  replan.request.req = *msg;
  if (!replan_srv_.call(replan)) {
    ROS_ERROR("%s: Replan server error.", name_.c_str());
    replan.response.traj.states.clear();
    replan.response.traj.times.clear();
  }

  // Publish trajectory.
  traj_pub_.publish(replan.response.traj);
}

// Initialize this class with all parameters and callbacks.
bool Replanner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Replanner");

  // Load parameters.
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  // Register callbacks.
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool Replanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/replan_request", replan_request_topic_)) return false;

  // Services.
  if (!nl.getParam("srv/replan", replan_srv_name_)) return false;

  return true;
}

// Register callbacks.
bool Replanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  traj_pub_ = nl.advertise<fastrack_msgs::Trajectory>(
    traj_topic_.c_str(), 1, false);

  // Subscribers.
  replan_request_sub_ = nl.subscribe(replan_request_topic_.c_str(), 1,
    &Replanner::ReplanRequestCallback, this);

  // Services.
  ros::service::waitForService(replan_srv_name_);
  replan_srv_ = nl.serviceClient<fastrack_srvs::Replan>(
    replan_srv_name_.c_str(), true);

  return true;
}

} //\namespace planning
} //\namespace fastrack
