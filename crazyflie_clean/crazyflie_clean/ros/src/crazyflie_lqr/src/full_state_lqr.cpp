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
// LQR hover controller for the Crazyflie. Assumes that the state space is
// given by the FullStateStamped message type, which is a 12D model.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_lqr/full_state_lqr.h>

namespace crazyflie_lqr {

// Register callbacks.
bool FullStateLqr::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &FullStateLqr::StateCallback, this);

  reference_sub_ = nl.subscribe(
    reference_topic_.c_str(), 1, &FullStateLqr::ReferenceCallback, this);

  // Control publisher.
  control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    control_topic_.c_str(), 1, false);

  return true;
}

// Process an incoming reference point change.
void FullStateLqr::ReferenceCallback(
  const crazyflie_msgs::FullStateStamped::ConstPtr& msg) {
  x_ref_(0) = msg->state.x;
  x_ref_(1) = msg->state.y;
  x_ref_(2) = msg->state.z;
  x_ref_(3) = msg->state.x_dot;
  x_ref_(4) = msg->state.y_dot;
  x_ref_(5) = msg->state.z_dot;
  x_ref_(6) = msg->state.roll;
  x_ref_(7) = msg->state.pitch;
  x_ref_(8) = msg->state.yaw;
  x_ref_(9) = msg->state.roll_dot;
  x_ref_(10) = msg->state.pitch_dot;
  x_ref_(11) = msg->state.yaw_dot;

  received_reference_ = true;
}

// Process an incoming state measurement.
void FullStateLqr::StateCallback(
  const crazyflie_msgs::FullStateStamped::ConstPtr& msg) {
  // Catch no reference.
  if (!received_reference_)
    return;

  // Read the message into the state and compute relative state.
  VectorXd x(x_dim_);
  x(0) = msg->state.x;
  x(1) = msg->state.y;
  x(2) = msg->state.z;
  x(3) = msg->state.x_dot;
  x(4) = msg->state.y_dot;
  x(5) = msg->state.z_dot;
  x(6) = msg->state.roll;
  x(7) = msg->state.pitch;
  x(8) = msg->state.yaw;
  x(9) = msg->state.roll_dot;
  x(10) = msg->state.pitch_dot;
  x(11) = msg->state.yaw_dot;

  VectorXd x_rel = x - x_ref_;

  // Rotate x and y coordinates.
  const double cos_yaw = std::cos(x(8));
  const double sin_yaw = std::sin(x(8));

  const double rot_x = cos_yaw * x_rel(0) + sin_yaw * x_rel(1);
  const double rot_y = -sin_yaw * x_rel(0) + cos_yaw * x_rel(1);
  const double rot_x_dot = cos_yaw * x_rel(3) + sin_yaw * x_rel(4);
  const double rot_y_dot = -sin_yaw * x_rel(3) + cos_yaw * x_rel(4);

  x_rel(0) = rot_x;
  x_rel(1) = rot_y;
  x_rel(3) = rot_x_dot;
  x_rel(4) = rot_y_dot;

  // Wrap angles.
  x_rel(6) = crazyflie_utils::angles::WrapAngleRadians(x_rel(6));
  x_rel(7) = crazyflie_utils::angles::WrapAngleRadians(x_rel(7));
  x_rel(8) = crazyflie_utils::angles::WrapAngleRadians(x_rel(8));

  // Compute optimal control.
  VectorXd u = K_ * x_rel + u_ref_;
  u(0) = crazyflie_utils::angles::WrapAngleRadians(u(0));
  u(1) = crazyflie_utils::angles::WrapAngleRadians(u(1));

  // Publish.
  crazyflie_msgs::ControlStamped control_msg;
  control_msg.control.roll = u(0);
  control_msg.control.pitch = u(1);
  control_msg.control.yaw_dot = u(2);
  control_msg.control.thrust = u(3);

  control_pub_.publish(control_msg);
}

} //\namespace crazyflie_lqr
