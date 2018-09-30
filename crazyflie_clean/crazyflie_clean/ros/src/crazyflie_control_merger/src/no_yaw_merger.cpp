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
// Class to merge control messages from two different controllers into
// a single ControlStamped message.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_control_merger/no_yaw_merger.h>

namespace crazyflie_control_merger {

// Register callbacks.
bool NoYawMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  prioritized_control_sub_ = nl.subscribe(
    prioritized_control_topic_.c_str(), 1, &NoYawMerger::NoYawControlCallback, this);

  return true;
}

// Process an incoming state measurement.
void NoYawMerger::NoYawControlCallback(
  const crazyflie_msgs::NoYawControlStamped::ConstPtr& msg) {
  no_yaw_control_ = msg->control;
  prioritized_control_been_updated_ = true;

  // Merge and publish.
  PublishMergedControl();
}

// Merge and publish.
void NoYawMerger::PublishMergedControl() const {
  crazyflie_msgs::ControlStamped msg;
  msg.header.stamp = ros::Time::now();

  if (!control_been_updated_) {
    return;
  } else if (!prioritized_control_been_updated_) {
    msg.control = control_;
  } else {
    // Extract no yaw priority.
    double p = no_yaw_control_.priority;
    if (mode_ == LQR)
      p = 0.0;
    else if (mode_ == PRIORITIZED)
      p = 1.0;

    // Set message fields.
    msg.control.roll = (1.0 - p) * control_.roll + p * no_yaw_control_.roll;
    msg.control.pitch = (1.0 - p) * control_.pitch + p * no_yaw_control_.pitch;
    msg.control.yaw_dot = control_.yaw_dot;
    msg.control.thrust = (1.0 - p) * control_.thrust + p * no_yaw_control_.thrust;
  }

  merged_pub_.publish(msg);
}

} //\namespace crazyflie_control_merger
