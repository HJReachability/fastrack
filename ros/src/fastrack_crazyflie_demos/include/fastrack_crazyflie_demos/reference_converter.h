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
// Since references are coming from the planner, this class needs to be
// templated on the planner state type.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CRAZYFLIE_DEMOS_REFERENCE_CONVERTER_H
#define FASTRACK_CRAZYFLIE_DEMOS_REFERENCE_CONVERTER_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <crazyflie_msgs/PositionVelocityStateStamped.h>
#include <fastrack_msgs/State.h>

#include <ros/ros.h>

namespace fastrack {
namespace crazyflie {

template <typename PS>
class ReferenceConverter : private Uncopyable {
 public:
  ~ReferenceConverter() {}
  explicit ReferenceConverter() : initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing new reference signals.
  void ReferenceCallback(const fastrack_msgs::State::ConstPtr& msg);

  // Publishers/subscribers and related topics.
  ros::Publisher raw_reference_pub_;
  ros::Subscriber fastrack_reference_sub_;

  std::string raw_reference_topic_;
  std::string fastrack_reference_topic_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

// ----------------------------- IMPLEMENTATION ---------------------------- //

// Initialize this class with all parameters and callbacks.
template <typename PS>
bool ReferenceConverter<PS>::Initialize(const ros::NodeHandle& n) {
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
template <typename PS>
bool ReferenceConverter<PS>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/fastrack_reference", fastrack_reference_topic_))
    return false;
  if (!nl.getParam("topic/raw_reference", raw_reference_topic_)) return false;

  return true;
}

// Register callbacks.
template <typename PS>
bool ReferenceConverter<PS>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriber.
  fastrack_reference_sub_ =
      nl.subscribe(fastrack_reference_topic_.c_str(), 1,
                   &ReferenceConverter::ReferenceCallback, this);

  // Publisher.
  raw_reference_pub_ =
      nl.advertise<crazyflie_msgs::PositionVelocityStateStamped>(
          raw_reference_topic_.c_str(), 1, false);

  return true;
}

// Callback for processing new reference signals.
template <typename PS>
void ReferenceConverter<PS>::ReferenceCallback(
    const fastrack_msgs::State::ConstPtr& msg) {
  // Convert to planner state type.
  const PS state(*msg);

  // Parse into Crazyflie msg.
  crazyflie_msgs::PositionVelocityStateStamped cf;
  cf.header.stamp = ros::Time::now();
  cf.state.x = state.X();
  cf.state.y = state.Y();
  cf.state.z = state.Z();
  cf.state.x_dot = state.Vx();
  cf.state.y_dot = state.Vy();
  cf.state.z_dot = state.Vz();

  raw_reference_pub_.publish(cf);
}

}  //\namespace crazyflie
}  //\namespace fastrack

#endif
