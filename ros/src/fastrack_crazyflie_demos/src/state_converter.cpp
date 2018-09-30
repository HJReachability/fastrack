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
// Defines the StateConverter class, which listens for new crazyflie state
// messages and immediately republishes them as fastrack state messages.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack_crazyflie_demos/state_converter.h>

namespace fastrack {
namespace crazyflie {

// Initialize this class with all parameters and callbacks.
bool StateConverter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "StateConverter");

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
bool StateConverter::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/fastrack_state", fastrack_state_topic_)) return false;
  if (!nl.getParam("topic/raw_state", raw_state_topic_)) return false;

  return true;
}

// Register callbacks.
bool StateConverter::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriber.
  raw_state_sub_ = nl.subscribe(raw_state_topic_.c_str(), 1,
    &StateConverter::StateCallback, this);

  // Publisher.
  fastrack_state_pub_ = nl.advertise<fastrack_msgs::State>(
    fastrack_state_topic_.c_str(), 1, false);

  return true;
}

// Callback for processing new state signals.
void StateConverter::
StateCallback(const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg) {
  fastrack_msgs::State s;
  s.x.push_back(msg->state.x);
  s.x.push_back(msg->state.y);
  s.x.push_back(msg->state.z);
  s.x.push_back(msg->state.x_dot);
  s.x.push_back(msg->state.y_dot);
  s.x.push_back(msg->state.z_dot);

  fastrack_state_pub_.publish(s);
}

} //\namespace crazyflie
} //\namespace fastrack
