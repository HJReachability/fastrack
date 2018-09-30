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
// Base class to merge control messages from two different controllers into
// a single ControlStamped message.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_control_merger/control_merger.h>

namespace crazyflie_control_merger {

// Initialize this node.
bool ControlMerger::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "control_merger");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Register extra callbacks for this base class.
  ros::NodeHandle nl(n);
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 1, &ControlMerger::ControlCallback, this);

  merged_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    merged_topic_.c_str(), 1, false);

  initialized_ = true;
  return true;
}

// Load parameters.
bool ControlMerger::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/prioritized_control", prioritized_control_topic_))
    return false;
  if (!nl.getParam("topics/merged", merged_topic_)) return false;

  // Mode.
  std::string mode;
  if (!nl.getParam("mode", mode)) return false;

  if (mode == "MERGE") {
    mode_ = MERGE;
    ROS_INFO("%s: Entering MERGE mode.", name_.c_str());
  } else if (mode == "LQR") {
    mode_ = LQR;
    ROS_INFO("%s: Entering LQR mode.", name_.c_str());
  } else if (mode == "PRIORITIZED") {
    mode_ = PRIORITIZED;
    ROS_INFO("%s: Entering PRIORITIZED mode.", name_.c_str());
  } else {
    ROS_ERROR("%s: Invalid mode. Using LQR.", name_.c_str());
    mode_ = LQR;
  }

  return true;
}

// Process an incoming reference point.
void ControlMerger::ControlCallback(
  const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  control_ = msg->control;
  control_been_updated_ = true;

  PublishMergedControl();
}

} //\namespace crazyflie_control_merger
