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
// Defines the PlannerManager class, which listens for new trajectory
// messages and, on a timer, repeatedly queries the current trajectory and
// publishes the corresponding reference.
//
// The PlannerManager is also responsible for requesting new plans.
// This base class only calls the planner once upon takeoff; as needs will vary
// derived classes may add further functionality such as receding
// horizon planning.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_PLANNER_MANAGER_H
#define FASTRACK_PLANNING_PLANNER_MANAGER_H

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
namespace planning {

using trajectory::Trajectory;

template<typename S>
class PlannerManager : private Uncopyable {
public:
  virtual ~PlannerManager() {}
  explicit PlannerManager()
    : ready_(false),
      waiting_for_traj_(false),
      serviced_updated_env_(false),
      initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

protected:
  // Load parameters and register callbacks. These may be overridden, however
  // derived classes should still call these functions.
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // If not already waiting, request a new trajectory that starts along the
  // current trajectory at the state corresponding to when the planner will
  // return, and ends at the goal location. This may be overridden by derived
  // classes with more specific replanning needs.
  virtual void MaybeRequestTrajectory();

  // Callback for applying tracking controller. This may be overridden, however
  // derived classes should still call thhis function.
  virtual void TimerCallback(const ros::TimerEvent& e);

  // Callback for processing trajectory updates.
  inline void TrajectoryCallback(const fastrack_msgs::Trajectory::ConstPtr& msg) {
    traj_ = Trajectory<S>(msg);
    traj_.Visualize(traj_vis_pub_, fixed_frame_);

    waiting_for_traj_ = false;
  }

  // Is the system ready?
  inline void ReadyCallback(const std_msgs::Empty::ConstPtr& msg) {
    ready_ = true;
  }

  // Generate a new trajectory request when environment has been updated.
  inline void UpdatedEnvironmentCallback(const std_msgs::Empty::ConstPtr& msg) {
    serviced_updated_env_ = false;
    MaybeRequestTrajectory();
  }

  // Current trajectory.
  Trajectory<S> traj_;

  // Planner runtime -- how long does it take for the planner to run.
  double planner_runtime_;

  // Are we waiting for a new trajectory?
  bool waiting_for_traj_;

  // Have we serviced the most recent updated environment callback?
  bool serviced_updated_env_;

  // Start/goal states.
  fastrack_msgs::State start_;
  fastrack_msgs::State goal_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Publishers/subscribers and related topics.
  ros::Publisher bound_pub_;
  ros::Publisher ref_pub_;
  ros::Publisher replan_request_pub_;
  ros::Publisher traj_vis_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber ready_sub_;
  ros::Subscriber updated_env_sub_;

  std::string bound_topic_;
  std::string ref_topic_;
  std::string replan_request_topic_;
  std::string traj_vis_topic_;
  std::string traj_topic_;
  std::string ready_topic_;
  std::string updated_env_topic_;

  // Frames of reference for publishing markers.
  std::string fixed_frame_;
  std::string planner_frame_;

  // Transform broadcaster for planner position.
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Are we in flight?
  bool ready_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

// ---------------------------- IMPLEMENTATION ------------------------------ //

// Initialize this class with all parameters and callbacks.
template<typename S>
bool PlannerManager<S>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PlannerManager");

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
bool PlannerManager<S>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/ready", ready_topic_)) return false;
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/ref", ref_topic_)) return false;
  if (!nl.getParam("topic/replan_request", replan_request_topic_)) return false;
  if (!nl.getParam("topic/updated_env", updated_env_topic_)) return false;
  if (!nl.getParam("vis/traj", traj_vis_topic_)) return false;
  if (!nl.getParam("vis/bound", bound_topic_)) return false;

  // Frames.
  if (!nl.getParam("frame/fixed", fixed_frame_)) return false;
  if (!nl.getParam("frame/planner", planner_frame_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  // Planner runtime, start, and goal.
  if (!nl.getParam("planner_runtime", planner_runtime_)) return false;
  if (!nl.getParam("start", start_.x)) return false;
  if (!nl.getParam("goal", goal_.x)) return false;

  return true;
}

// Register callbacks.
template<typename S>
bool PlannerManager<S>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  ready_sub_ = nl.subscribe(ready_topic_.c_str(), 1,
    &PlannerManager<S>::ReadyCallback, this);

  traj_sub_ = nl.subscribe(traj_topic_.c_str(), 1,
    &PlannerManager<S>::TrajectoryCallback, this);

  updated_env_sub_ = nl.subscribe(updated_env_topic_.c_str(), 1,
    &PlannerManager<S>::UpdatedEnvironmentCallback, this);

  // Publishers.
  ref_pub_ = nl.advertise<fastrack_msgs::State>(ref_topic_.c_str(), 1, false);

  replan_request_pub_ = nl.advertise<fastrack_msgs::ReplanRequest>(
    replan_request_topic_.c_str(), 1, false);

  bound_pub_ = nl.advertise<visualization_msgs::Marker>(
    bound_topic_.c_str(), 1, false);

  traj_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    traj_vis_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_),
    &PlannerManager<S>::TimerCallback, this);

  return true;
}

// If not already waiting, request a new trajectory that starts along the
// current trajectory at the state corresponding to when the planner will
// return, and ends at the goal location. This may be overridden by derived
// classes with more specific replanning needs.
template<typename S>
void PlannerManager<S>::MaybeRequestTrajectory() {
  if (!ready_ || waiting_for_traj_) {
    return;
  }

  // Set start and goal states.
  fastrack_msgs::ReplanRequest msg;
  msg.start = start_;
  msg.goal = goal_;

  // Set start time.
  msg.start_time = ros::Time::now().toSec() + planner_runtime_;

  // Reset start state for future state if we have a current trajectory.
  if (traj_.Size() > 0) {
    // Catch trajectory that's too short.
    if (traj_.LastTime() < msg.start_time) {
      ROS_ERROR("%s: Current trajectory is too short. Cannot interpolate.",
                name_.c_str());
      msg.start = traj_.LastState().ToRos();
    } else {
      msg.start = traj_.Interpolate(msg.start_time).ToRos();
    }
  }

  // Publish request and set flag.
  replan_request_pub_.publish(msg);
  waiting_for_traj_ = true;
  serviced_updated_env_ = true;
}

// Callback for applying tracking controller.
template<typename S>
void PlannerManager<S>::TimerCallback(const ros::TimerEvent& e) {
  if (!ready_)
    return;

  if (traj_.Size() == 0) {
    MaybeRequestTrajectory();
    return;
  } else if (waiting_for_traj_) {
    ROS_WARN_THROTTLE(1.0, "%s: Waiting for trajectory.", name_.c_str());
  } else if (!serviced_updated_env_) {
    ROS_INFO_THROTTLE(1.0, "%s: Servicing old updated environment callback.",
                      name_.c_str());
    MaybeRequestTrajectory();
  }

  // Interpolate the current trajectory.
  const S planner_x = traj_.Interpolate(ros::Time::now().toSec());

  // Convert to ROS msg and publish.
  ref_pub_.publish(planner_x.ToRos());

  // Broadcast transform.
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = fixed_frame_;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = planner_frame_;

  tf.transform.translation.x = planner_x.X();
  tf.transform.translation.y = planner_x.Y();
  tf.transform.translation.z = planner_x.Z();
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;

  tf_broadcaster_.sendTransform(tf);
}

} //\namespace planning
} //\namespace fastrack

#endif
