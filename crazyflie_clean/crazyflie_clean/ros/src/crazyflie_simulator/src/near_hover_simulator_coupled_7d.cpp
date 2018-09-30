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
// Near hover simulator.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_simulator/near_hover_simulator_coupled_7d.h>
#include <crazyflie_utils/angles.h>

namespace crazyflie_simulator {

// Initialize this class by reading parameters and loading callbacks.
bool NearHoverSimulatorCoupled7D::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "near_hover_simulator");

  // Set state and control to zero initially.
  x_ = VectorXd::Zero(7);
  u_ = VectorXd::Zero(4);
  //  u_(3) = crazyflie_utils::constants::G;

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set initial time.
  last_time_ = ros::Time::now();

  initialized_ = true;
  return true;
}

// Load parameters and register callbacks.
bool NearHoverSimulatorCoupled7D::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Frames of reference.
  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/robot", robot_frame_id_)) return false;

  // Time step for reading tf.
  if (!nl.getParam("time_step", dt_)) return false;

  // Control topic.
  if (!nl.getParam("topics/control", control_topic_)) return false;

  // Get initial position.
  double init_x, init_y, init_z;
  if (!nl.getParam("init/x", init_x)) return false;
  if (!nl.getParam("init/y", init_y)) return false;
  if (!nl.getParam("init/z", init_z)) return false;

  x_(0) = init_x;
  x_(1) = init_y;
  x_(2) = init_z;

  return true;
}

bool NearHoverSimulatorCoupled7D::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 1, &NearHoverSimulatorCoupled7D::ControlCallback, this);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(dt_), &NearHoverSimulatorCoupled7D::TimerCallback, this);

  return true;
}

// Timer callback.
void NearHoverSimulatorCoupled7D::TimerCallback(const ros::TimerEvent& e) {
  const ros::Time now = ros::Time::now();

  // Only update state if we have received a control signal from outside.
  if (received_control_)
    x_ += dynamics_(x_, u_) * (now - last_time_).toSec();

  // Threshold at ground!
  if (x_(2) < 0.0) {
    // HACK! Assuming state layout.
    x_(2) = 0.0;
    x_(5) = std::max(0.0, x_(5));
  }

  // Update last time.
  last_time_ = now;

  // Broadcast on tf.
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = fixed_frame_id_;
  transform_stamped.header.stamp = now;

  transform_stamped.child_frame_id = robot_frame_id_;

  transform_stamped.transform.translation.x = x_(0);
  transform_stamped.transform.translation.y = x_(1);
  transform_stamped.transform.translation.z = x_(2);

  // RPY to quaternion.
  const double roll = u_(0);
  const double pitch = u_(1);
  const double yaw = x_(6);
  const Quaterniond q =
    Eigen::AngleAxisd(roll, Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Vector3d::UnitZ());

  /**
  Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  euler(0) = angles::WrapAngleRadians(euler(0));
  euler(1) = angles::WrapAngleRadians(euler(1));
  euler(2) = angles::WrapAngleRadians(euler(2));

  std::cout << "roll = " << u_(0) << ", q roll = " << euler(0) << std::endl;
  std::cout << "pitch = " << u_(1) << ", q pitch = " << euler(1) << std::endl;
  std::cout << "yaw = " << u_(2) << ", q yaw = " << euler(2) << std::endl;
  **/

  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  br_.sendTransform(transform_stamped);
}

// Update control signal.
void NearHoverSimulatorCoupled7D::ControlCallback(
  const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  u_(0) = msg->control.roll;
  u_(1) = msg->control.pitch;
  u_(2) = msg->control.yaw_dot;
  u_(3) = msg->control.thrust;

  received_control_ = true;
}

} //\namespace crazyflie_simulator
