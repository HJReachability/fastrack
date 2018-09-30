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
// Class to convert ControlStamped messages to Twists and publish on /cmd_vel.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_control_merger/cmd_vel_converter.h>

namespace crazyflie_control_merger {

// Initialize this node.
bool CmdVelConverter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "cmd_vel_converter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool CmdVelConverter::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/cmd_vel", cmd_vel_topic_)) return false;

  return true;
}

// Register callbacks.
bool CmdVelConverter::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 1, &CmdVelConverter::ControlCallback, this);

  // Publishers.
  cmd_vel_pub_ = nl.advertise<geometry_msgs::Twist>(
    cmd_vel_topic_.c_str(), 1, false);

  return true;
}
// Process an incoming reference point.
void CmdVelConverter::
ControlCallback(const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  geometry_msgs::Twist twist;

  // Fill in the Twist, following the conversion process in the
  // crazyflie_server.cpp file function named "cmdVelChanged()".
  // NOTE! Some dimensions have been flipped and/or converted to degrees
  // in order to conform to expectations of the crazyflie firmware.
  twist.linear.y = crazyflie_utils::angles::RadiansToDegrees(msg->control.roll);
  twist.linear.x = crazyflie_utils::angles::RadiansToDegrees(msg->control.pitch);
  twist.angular.z = -crazyflie_utils::angles::RadiansToDegrees(msg->control.yaw_dot);
  twist.linear.z = crazyflie_utils::pwm::ThrustToPwmDouble(msg->control.thrust);

  cmd_vel_pub_.publish(twist);
}

} //\namespace crazyflie_control_merger
