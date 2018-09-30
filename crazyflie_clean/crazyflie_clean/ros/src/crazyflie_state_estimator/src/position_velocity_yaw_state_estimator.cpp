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
// Derived from StateEstimator, for the 7D PositionVelocityYawState message.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_state_estimator/position_velocity_yaw_state_estimator.h>

// Register callbacks.
bool PositionVelocityYawStateEstimator::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // State publisher.
  state_pub_ = nl.advertise<crazyflie_msgs::PositionVelocityYawStateStamped>(
    state_topic_.c_str(), 1, false);

  return true;
}

// Merge a pose measured at the given time (specified by translation
// and euler angles) into the current state estimate.
void PositionVelocityYawStateEstimator::Update(const Vector3d& translation,
                                               const Vector3d& euler,
                                               const ros::Time& stamp) {
  // Catch first update.
  if (first_update_) {
    x_(0) = translation(0);
    x_(1) = translation(1);
    x_(2) = translation(2);

    x_(3) = 0.0;
    x_(4) = 0.0;
    x_(5) = 0.0;

    x_(6) = euler(2);

    first_update_ = false;
  } else {
    // Time difference.
    const double dt = (stamp - last_time_).toSec();

    // TODO! Use a smoothing filter here instead.
    // Update velocities.
    x_(3) = (translation(0) - x_(0)) / dt;
    x_(4) = (translation(1) - x_(1)) / dt;
    x_(5) = (translation(2) - x_(2)) / dt;

    // Update position/orientation.
    x_(0) = translation(0);
    x_(1) = translation(1);
    x_(2) = translation(2);

    x_(6) = euler(2);
  }

  // Publish.
  crazyflie_msgs::PositionVelocityYawStateStamped msg;

  msg.header.frame_id = fixed_frame_id_;
  msg.header.stamp = stamp;

  msg.state.x = x_(0);
  msg.state.y = x_(1);
  msg.state.z = x_(2);
  msg.state.x_dot = x_(3);
  msg.state.y_dot = x_(4);
  msg.state.z_dot = x_(5);
  msg.state.yaw = x_(6);

  state_pub_.publish(msg);
}
