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
// Class to filter control messages coming from the takeoff server from other
// sources of control signals so that the takeoff server takes precedence.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_takeoff/takeoff_control_filter.h>

namespace crazyflie_takeoff {

// Initialize this node.
bool TakeoffControlFilter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "takeoff_control_filter");

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
bool TakeoffControlFilter::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topics/final_control", final_control_topic_)) return false;
  if (!nl.getParam("topics/takeoff_control", takeoff_control_topic_))
    return false;
  if (!nl.getParam("topics/commanded_control", commanded_control_topic_))
    return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;

  return true;
}

// Register callbacks.
bool TakeoffControlFilter::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriberrs.
  takeoff_control_sub_ =
    nl.subscribe(takeoff_control_topic_.c_str(), 1,
                 &TakeoffControlFilter::TakeoffControlCallback, this);

  commanded_control_sub_ =
    nl.subscribe(commanded_control_topic_.c_str(), 1,
                 &TakeoffControlFilter::CommandedControlCallback, this);

  in_flight_sub_ =
    nl.subscribe(in_flight_topic_.c_str(), 1,
                 &TakeoffControlFilter::InFlightCallback, this);

  // Publishers.
  final_control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    final_control_topic_.c_str(), 10, false);

  return true;
}

// Callback for takeoff server controls.
void TakeoffControlFilter::
TakeoffControlCallback(const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  received_takeoff_control_ = true;
  takeoff_control_ = *msg;

  // Takeoff controller always takes precedence.
  final_control_pub_.publish(takeoff_control_);
}

// Callback for commanded controls.
void TakeoffControlFilter::
CommandedControlCallback(const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  // If we're in flight then the commanded control takes precedence.
  if (in_flight_)
    final_control_pub_.publish(*msg);

  // If the most recent takeoff control is old, then the commanded control
  // takes precedence.
  else if (!received_takeoff_control_ ||
           (ros::Time::now() - takeoff_control_.header.stamp).toSec() > 0.5)
    final_control_pub_.publish(*msg);

  // Otherwise send the takeoff control.
  else
    final_control_pub_.publish(takeoff_control_);
}

// In flight signal callback.
void TakeoffControlFilter::
InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
  in_flight_ = !in_flight_;
}

} //\namespace crazyflie_takeoff
