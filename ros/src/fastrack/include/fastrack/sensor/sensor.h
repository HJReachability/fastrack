/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
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
// Base class for all sensor models. Sensors are used in simulation to generate
// "fake" sensor measurements from a known environment.
//
// Sensors are templated on the type of environment (E), on the type of
// message (M) that they publish, and on the parameter struct (P) used to
// query the environment for sensor messages.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_SENSOR_SENSOR_H
#define FASTRACK_SENSOR_SENSOR_H

#include <fastrack/utils/types.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

namespace fastrack {
namespace sensor {

template<typename E, typename M, typename P>
class Sensor {
public:
  virtual ~Sensor() {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

protected:
  explicit Sensor()
    : tf_listener_(tf_buffer_),
      initialized_(false) {}

  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via Sensor::LoadParameters).
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Register callbacks.
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Every time the timer fires, generate a new sensor measurement.
  void TimerCallback(const ros::TimerEvent& e);

  // Update sensor parameters.
  virtual void UpdateParameters() = 0;

  // Derived classes must have some sort of visualization through RViz.
  virtual void Visualize() const = 0;

  // Ground truth environment.
  E env_;

  // Parameters.
  P params_;

  // Publishers.
  ros::Publisher vis_pub_;
  ros::Publisher sensor_pub_;

  std::string vis_topic_;
  std::string sensor_topic_;

  // Frames of reference.
  std::string sensor_frame_;
  std::string fixed_frame_;

  // Buffer and listener to get current sensor pose.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Timer.
  ros::Timer timer_;
  double time_step_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
}; //\class Sensor

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Initialize from a ROS NodeHandle.
template<typename E, typename M, typename P>
bool Sensor<E, M, P>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Sensor");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize the ground truth environment.
  if (!env_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize environment.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via Sensor::LoadParameters).
template<typename E, typename M, typename P>
bool Sensor<E, M, P>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/sensor", sensor_topic_)) return false;
  if (!nl.getParam("vis/sensor", vis_topic_)) return false;

  // Frames of reference.
  if (!nl.getParam("frame/fixed", fixed_frame_)) return false;
  if (!nl.getParam("frame/sensor", sensor_frame_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  return true;
}

// Register callbacks.
template<typename E, typename M, typename P>
bool Sensor<E, M, P>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  sensor_pub_ = nl.advertise<M>(
    sensor_topic_.c_str(), 1, false);

  vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    vis_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Sensor<E, M, P>::TimerCallback, this);

  return true;
}

// Every time the timer fires, generate a new sensor measurement.
template<typename E, typename M, typename P>
void Sensor<E, M, P>::TimerCallback(const ros::TimerEvent& e) {
  // Get most up-to-date parameters (e.g. by reading pose from TF).
  UpdateParameters();

  // Query environment and publish.
  sensor_pub_.publish(env_.SimulateSensor(params_));
}

} //\namespace sensor
} //\namespace fastrack

#endif
