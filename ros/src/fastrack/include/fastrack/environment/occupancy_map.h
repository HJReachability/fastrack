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
// Base class for all occupancy maps, which are types of environment models
// in which each point in space is assigned a probability of being occupied.
// The base IsValid check is then a threshold test on the corresponding
// occupancy probability.
//
// Like Environment, this class is templated on the sensor message type (M)
// and the sensor parameters type (P) which may be used to simulate sensor
// measurements.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_ENVIRONMENT_ENVIRONMENT_H
#define FASTRACK_ENVIRONMENT_ENVIRONMENT_H

#include <fastrack/bound/box.h>
#include <fastrack/utils/types.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

namespace fastrack {
namespace environment {

using bound::Box;

template <typename M, typename P> class OccupancyMap : public Environment {
public:
  virtual ~OccupancyMap() {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle &n);

  // Collision check is a threshold test on occupancy probability integrated
  // over the bound.
  bool IsValid(const Vector3d &position, const Box &bound) const {
    return initialized_ &&
           OccupancyProbability(position, bound) < free_space_threshold_;
  }

  // Derived classes must provide an OccupancyProbability function for both
  // single points and tracking error bounds centered on a point.
  virtual double OccupancyProbability(const Vector3d &position) const = 0;
  virtual double OccupancyProbability(const Vector3d &position,
                                      const Box &bound) const = 0;

  // Generate a sensor measurement.
  virtual M SimulateSensor(const P &params) const = 0;

  // Derived classes must have some sort of visualization through RViz.
  virtual void Visualize() const = 0;

protected:
  explicit OccupancyMap() : Environment() {}

  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via OccupancyMap::LoadParameters).
  virtual bool LoadParameters(const ros::NodeHandle &n);

  // Update this environment with the information contained in the given
  // sensor measurement.
  // NOTE! This function needs to publish on `updated_topic_`.
  virtual void SensorCallback(const typename M::ConstPtr &msg) = 0;

  // Occupancy probability threshold below which a point/region is considered
  // to be free space.
  double free_space_threshold_;
}; //\class Environment

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via OccupancyMap::LoadParameters).
template <typename M, typename P>
bool OccupancyMap<M, P>::LoadParameters(const ros::NodeHandle &n) {
  if (!Environment<M, P>::LoadParameters(n))
    return false;

  ros::NodeHandle nl(n);

  // Free space threshold.
  if (!nl.getParam("free_space_threshold", free_space_threshold_))
    return false;

  return true;
}

} //\namespace environment
} //\namespace fastrack

#endif
