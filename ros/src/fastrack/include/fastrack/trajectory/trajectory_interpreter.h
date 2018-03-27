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
// Defines the TrajectoryInterpreter class, which listens for new trajectory
// messages and, on a timer, repeatedly queries the current trajectory and
// publishes the corresponding reference.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_TRAJECTORY_INTERPRETER_H
#define FASTRACK_TRAJECTORY_INTERPRETER_H

#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <fastrack_msgs/ReplanRequest.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace fastrack {
namespace trajectory {

template<typename S>
class TrajectoryInterpreter : private Uncopyable {
public:
  ~TrajectoryInterpreter() {}
  explicit TrajectoryInterpreter()
    : initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for applying tracking controller.
  void TimerCallback(const ros::TimerEvent& e);

  // Callback for processing trajectory updates.
  inline void TrajectoryCallback(const fastrack_msgs::Trajectory::ConstPtr& msg) {
    traj_ = Trajectory<S>(msg);
  }

  // Process in flight notifications.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }

  // Current trajectory.
  Trajectory<S> traj_;

  // Planner runtime -- how long does it take for the planner to run.
  double planner_runtime_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Publishers/subscribers and related topics.
  ros::Publisher bound_pub_;
  ros::Publisher ref_pub_;
  ros::Publisher request_traj_pub_;
  ros::Publisher traj_vis_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber in_flight_sub_;

  std::string bound_topic_;
  std::string ref_topic_;
  std::string request_traj_topic_;
  std::string traj_vis_topic_;
  std::string traj_topic_;
  std::string in_flight_topic_;

  // Frames of reference for publishing markers.
  std::string fixed_frame_id_;
  std::string planner_frame_id_;

  // Transform broadcaster for planner position.
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Are we in flight?
  bool in_flight_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

// ---------------------------- IMPLEMENTATION ------------------------------ //

// Initialize this class with all parameters and callbacks.
template<typename S>
bool TrajectoryInterpreter<S>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "TrajectoryInterpreter");

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
template<typename S>
bool TrajectoryInterpreter<S>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/ref", ref_topic_)) return false;
  if (!nl.getParam("topic/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("vis/traj", traj_vis_topic_)) return false;
  if (!nl.getParam("vis/bound", bound_topic_)) return false;

  // Frames.
  if (!nl.getParam("frame/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frame/planner", planner_frame_id_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  return true;
}

// Register callbacks.
template<typename S>
bool TrajectoryInterpreter<S>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  in_flight_sub_ = nl.subscribe(in_flight_topic_.c_str(), 1,
    &TrajectoryInterpreter<S>::InFlightCallback, this);

  traj_sub_ = nl.subscribe(traj_topic_.c_str(), 1,
    &TrajectoryInterpreter<S>::TrajectoryCallback, this);

  // Publishers.
  ref_pub_ = nl.advertise<fastrack_msgs::State>(ref_topic_.c_str(), 1, false);

  request_traj_pub_ = nl.advertise<fastrack_msgs::ReplanRequest>(
    request_traj_topic_.c_str(), 1, false);

  bound_pub_ = nl.advertise<visualization_msgs::Marker>(
    bound_topic_.c_str(), 1, false);

  traj_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    traj_vis_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_),
    &TrajectoryInterpreter<S>::TimerCallback, this);

  return true;
}

// Callback for applying tracking controller.
template<typename S>
void TrajectoryInterpreter<S>::TimerCallback(const ros::TimerEvent& e) {
  if (traj_.Size() == 0) {
    ROS_WARN_THROTTLE(1.0, "%s: No trajectory received.", name_.c_str());
    return;
  }

  // Interpolate the current trajectory.
  const S planner_x = traj_.Interpolate(ros::Time::now().toSec());

  // Convert to ROS msg and publish.
  ref_pub_.publish(planner_x.ToRos());
}

} //\namespace trajectory
} //\namespace fastrack

#endif
