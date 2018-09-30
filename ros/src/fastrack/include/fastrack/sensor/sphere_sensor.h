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
// Sensor to detect spherical obstacles. Templated on environment type.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_SENSOR_SPHERE_SENSOR_H
#define FASTRACK_SENSOR_SPHERE_SENSOR_H

#include <fastrack/sensor/sensor.h>
#include <fastrack/sensor/sphere_sensor_params.h>
#include <fastrack_msgs/SensedSpheres.h>

namespace fastrack {
namespace sensor {

template<typename E>
class SphereSensor : public Sensor<
  E, fastrack_msgs::SensedSpheres, SphereSensorParams> {
public:
  ~SphereSensor() {}
  explicit SphereSensor()
    : Sensor<E, fastrack_msgs::SensedSpheres, SphereSensorParams>() {}

private:
  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via Sensor::LoadParameters).
  bool LoadParameters(const ros::NodeHandle& n);

  // Update sensor parameters.
  void UpdateParameters();

  // Derived classes must have some sort of visualization through RViz.
  void Visualize() const;
}; //\class SphereSensor

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via Sensor::LoadParameters).
template<typename E>
bool SphereSensor<E>::LoadParameters(const ros::NodeHandle& n) {
  if (!Sensor<E, fastrack_msgs::SensedSpheres, SphereSensorParams>::
      LoadParameters(n))
    return false;

  ros::NodeHandle nl(n);

  // Range.
  if (!nl.getParam("range", this->params_.range)) return false;

  return true;
}

// Update sensor parameters.
template<typename E>
void SphereSensor<E>::UpdateParameters() {
  // Get the current sensor pose from tf.
  geometry_msgs::TransformStamped tf;

  try {
    tf = this->tf_buffer_.lookupTransform(
      this->fixed_frame_.c_str(), this->sensor_frame_.c_str(), ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", this->name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current sensor pose.", this->name_.c_str());
    return;
  }

  // Unpack into parameters struct.
  this->params_.position(0) = tf.transform.translation.x;
  this->params_.position(1) = tf.transform.translation.y;
  this->params_.position(2) = tf.transform.translation.z;

  return;
}

// Derived classes must have some sort of visualization through RViz.
template<typename E>
void SphereSensor<E>::Visualize() const {
  visualization_msgs::Marker m;
  m.ns = "sensor";
  m.header.frame_id = this->sensor_frame_;
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.color.a = 0.3;
  m.color.r = 0.1;
  m.color.g = 0.5;
  m.color.b = 0.3;
  m.scale.x = 2.0 * this->params_.range;
  m.scale.y = 2.0 * this->params_.range;
  m.scale.z = 2.0 * this->params_.range;

  this->vis_pub_.publish(m);
}

} //\namespace sensor
} //\namespace fastrack

#endif
