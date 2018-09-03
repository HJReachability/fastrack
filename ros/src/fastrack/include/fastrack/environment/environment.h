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
// Base class for all environment models, providing separate collision check
// functions for each type of tracking error bound. All environments are
// boxes in 3D space. Environment is templated on the specific type of sensor
// message (M) which may be generated from or incorporated into a derived class,
// and sensor parameters (P) which may be used to generate sensor readings.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_ENVIRONMENT_ENVIRONMENT_H
#define FASTRACK_ENVIRONMENT_ENVIRONMENT_H

#include <fastrack/bound/box.h>
#include <fastrack/bound/sphere.h>
#include <fastrack/utils/types.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

namespace fastrack {
namespace environment {

using bound::Box;
using bound::Sphere;

template <typename M, typename P>
class Environment {
 public:
  virtual ~Environment() {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided position is a valid collision-free configuration.
  // Provide a separate collision check for each type of tracking error bound.
  virtual bool IsValid(
      const Vector3d& position, const Box& bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const = 0;
  virtual bool IsValid(
      const Vector3d& position, const Sphere& bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const = 0;

  // Utility for checking multiple positions.
  template <typename B>
  bool AreValid(const std::vector<Vector3d>& positions, const B& bound,
                double time = std::numeric_limits<double>::quiet_NaN()) const {
    // Return Boolean AND of all IsValid calls.
    for (const auto& p : positions) {
      if (!IsValid(p, bound, time)) return false;
    }

    return true;
  }

  // Generate a sensor measurement.
  virtual M SimulateSensor(const P& params) const = 0;

  // Derived classes must have some sort of visualization through RViz.
  virtual void Visualize() const = 0;

 protected:
  explicit Environment() : initialized_(false) {}

  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via Environment::LoadParameters).
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Register callbacks.
  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // Update this environment with the information contained in the given
  // sensor measurement.
  // NOTE! This function needs to publish on `updated_topic_`.
  virtual void SensorCallback(const typename M::ConstPtr& msg) = 0;

  // Upper and lower bounds.
  Vector3d lower_;
  Vector3d upper_;

  // Publishers and subscribers.
  ros::Publisher vis_pub_;
  ros::Publisher updated_pub_;
  ros::Subscriber sensor_sub_;

  std::string vis_topic_;
  std::string updated_topic_;
  std::string sensor_topic_;

  // Frame in which to publish visualization.
  std::string fixed_frame_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};  //\class Environment

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Initialize from a ROS NodeHandle.
template <typename M, typename P>
bool Environment<M, P>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Environment");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Visualize.
  Visualize();

  initialized_ = true;
  return true;
}

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via Environment::LoadParameters).
template <typename M, typename P>
bool Environment<M, P>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Sensor topic/service.
  if (!nl.getParam("topic/sensor_sub", sensor_topic_)) return false;
  if (!nl.getParam("topic/updated_env", updated_topic_)) return false;
  if (!nl.getParam("vis/env", vis_topic_)) return false;

  // Frame of reference to publish visualization in.
  if (!nl.getParam("frame/fixed", fixed_frame_)) return false;

  // Upper and lower bounds of the environment.
  if (!nl.getParam("env/upper/x", upper_(0))) return false;
  if (!nl.getParam("env/upper/y", upper_(1))) return false;
  if (!nl.getParam("env/upper/z", upper_(2))) return false;

  if (!nl.getParam("env/lower/x", lower_(0))) return false;
  if (!nl.getParam("env/lower/y", lower_(1))) return false;
  if (!nl.getParam("env/lower/z", lower_(2))) return false;

  return true;
}

// Register callbacks.
template <typename M, typename P>
bool Environment<M, P>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  sensor_sub_ = nl.subscribe(sensor_topic_.c_str(), 1,
                             &Environment<M, P>::SensorCallback, this);

  // Publishers.
  vis_pub_ =
      nl.advertise<visualization_msgs::Marker>(vis_topic_.c_str(), 1, false);

  updated_pub_ =
      nl.advertise<std_msgs::Empty>(updated_topic_.c_str(), 1, false);

  return true;
}

}  //\namespace environment
}  //\namespace fastrack

#endif
