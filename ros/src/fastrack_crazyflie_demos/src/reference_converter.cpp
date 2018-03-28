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
// Defines the ReferenceConverter class, which listens for new fastrack state
// messages and immediately republishes them as crazyflie messages.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack_crazyflie_demos/reference_converter.h>

namespace fastrack {
namespace crazyflie {

// Initialize this class with all parameters and callbacks.
bool ReferenceConverter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "ReferenceConverter");

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
bool ReferenceConverter::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/fastrack_reference", fastrack_reference_topic_)) return false;
  if (!nl.getParam("topic/raw_reference", raw_reference_topic_)) return false;

  return true;
}

// Register callbacks.
bool ReferenceConverter::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriber.
  fastrack_reference_sub_ = nl.subscribe(fastrack_reference_topic_.c_str(), 1,
    &ReferenceConverter::ReferenceCallback, this);

  // Publisher.
  raw_reference_pub_ = nl.advertise<crazyflie_msgs::PositionVelocityStateStamped>(
    raw_reference_topic_.c_str(), 1, false);

  return true;
}

// Callback for processing new reference signals.
void ReferenceConverter::
ReferenceCallback(const fastrack_msgs::State::ConstPtr& msg) {
  if (msg->x.size() != 6) {
    ROS_ERROR("%s: FaSTrack reference msg of incorrect dimension.",
              name_.c_str());
    return;
  }

  crazyflie_msgs::PositionVelocityStateStamped cf;
  cf.header.stamp = ros::Time::now();
  cf.state.x = msg->x[0];
  cf.state.y = msg->x[1];
  cf.state.z = msg->x[2];
  cf.state.x_dot = msg->x[3];
  cf.state.y_dot = msg->x[4];
  cf.state.z_dot = msg->x[5];

  raw_reference_pub_.publish(cf);
}

} //\namespace crazyflie
} //\namespace fastrack
