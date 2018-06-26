/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
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
// Base class for all planners. Planners are templated on state and dynamics
// types **of the planner** NOT **of the tracker,** as well as the tracking
// error bound type and environment. Planners take in start and goal states, and
// start time, and output a trajectory of planner states.
//
// Templated on state (S), environment (E), dynamics (D), dynamics service (SD),
// bound (B), and bound service (SB).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_PLANNER_H
#define FASTRACK_PLANNING_PLANNER_H

#include <fastrack/environment/environment.h>
#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>

#include <fastrack_msgs/ReplanRequest.h>
#include <fastrack_msgs/Trajectory.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace fastrack {
namespace planning {

using environment::Environment;
using trajectory::Trajectory;

template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
class Planner {
public:
  virtual ~Planner() {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

protected:
  explicit Planner()
    : initialized_(false) {}

  // Load parameters and register callbacks. These may be overridden
  // by derived classes if needed (they should still call these functions
  // via Planner::LoadParameters and Planner::RegisterCallbacks).
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback to handle replanning requests.
  inline void ReplanRequestCallback(
    const fastrack_msgs::ReplanRequest::ConstPtr& msg) const {
    // Unpack start/stop states.
    const S start(msg->start);
    const S goal(msg->goal);

    // Plan and publish.
    traj_pub_.publish(Plan(start, goal, msg->start_time).ToRos());
  }

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  virtual Trajectory<S> Plan(
    const S& start, const S& goal, double start_time=0.0) const = 0;

  // Keep a copy of the dynamics, tracking bound, and environment.
  D dynamics_;
  B bound_;
  E env_;

  // Max amount of time for planning to run each time.
  double max_runtime_;

  // State space bounds.
  std::vector<double> state_upper_;
  std::vector<double> state_lower_;

  // Publishers and subscriber.
  ros::Subscriber replan_request_sub_;
  ros::Publisher traj_pub_;

  std::string replan_request_topic_;
  std::string traj_topic_;

  // Services for loading dynamics and bound.
  ros::ServiceClient dynamics_srv_;
  ros::ServiceClient bound_srv_;

  std::string dynamics_srv_name_;
  std::string bound_srv_name_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
}; //\class Planner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Initialize from a ROS NodeHandle.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
bool Planner<S, E, D, SD, B, SB>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Planner");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize environment.
  if (!env_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize environment.", name_.c_str());
    return false;
  }

  // Set bound by calling service provided by tracker.
  if (!bound_srv_) {
    ROS_ERROR("%s: Bound server was disconnected.", name_.c_str());
    return false;
  }

  SB b;
  if (!bound_srv_.call(b)) {
    ROS_ERROR("%s: Bound server error.", name_.c_str());
    return false;
  }

  bound_.FromRos(b.response);

  // Set dynamics by calling service provided by tracker.
  if (!dynamics_srv_) {
    ROS_ERROR("%s: Dynamics server was disconnected.", name_.c_str());
    return false;
  }

  SD d;
  if (!dynamics_srv_.call(d)) {
    ROS_ERROR("%s: Dynamics server error.", name_.c_str());
    return false;
  }

  dynamics_.FromRos(d.response);

  // Set configuration space bounds.
  S::SetBounds(state_lower_, state_upper_);

  initialized_ = true;
  return true;
}

// Load parameters.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
bool Planner<S, E, D, SD, B, SB>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/replan_request", replan_request_topic_))
    return false;

  // Services.
  if (!nl.getParam("srv/dynamics", dynamics_srv_name_)) return false;
  if (!nl.getParam("srv/bound", bound_srv_name_)) return false;

  // Max runtime per call.
  if (!nl.getParam("max_runtime", max_runtime_)) return false;

  // State space bounds.
  // HACK! For some reason fails to load here so hard-coding defaults for now.
  if (!nl.getParam("state/lower", state_lower_)) //return false;
    state_lower_ = { -10.0, -10.0, 0.0, -10.0, -10.0, -10.0 };
  if (!nl.getParam("state/upper", state_upper_)) //return false;
    state_upper_ = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };

  return true;
}

// Register callbacks.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
bool Planner<S, E, D, SD, B, SB>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  replan_request_sub_ = nl.subscribe(
    replan_request_topic_.c_str(), 1, &Planner::ReplanRequestCallback, this);

  // Publishers.
  traj_pub_ = nl.advertise<fastrack_msgs::Trajectory>(
    traj_topic_.c_str(), 1, false);

  // Services.
  ros::service::waitForService(dynamics_srv_name_);
  dynamics_srv_ = nl.serviceClient<SD>(dynamics_srv_name_.c_str(), true);

  ros::service::waitForService(bound_srv_name_);
  bound_srv_ = nl.serviceClient<SB>(bound_srv_name_.c_str(), true);

  return true;
}

} //\namespace planning
} //\namespace fastrack

#endif
