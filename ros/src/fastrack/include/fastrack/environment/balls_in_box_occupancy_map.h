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
// BallsInBoxOccupancyMap is derived from OccupancyMap, and models space as
// a collection of spherical sensor fields of view and spherical obstacles,
// which may overlap.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_ENVIRONMENT_BALLS_IN_BOX_OCCUPANCY_MAP_H
#define FASTRACK_ENVIRONMENT_BALLS_IN_BOX_OCCUPANCY_MAP_H

#include <fastrack/environment/occupancy_map.h>
#include <fastrack/sensor/sphere_sensor.h>
#include <fastrack/sensor/sphere_sensor_params.h>
#include <fastrack/utils/kdtree_map.h>
#include <fastrack_msgs/SensedSpheres.h>

namespace fastrack {
namespace environment {

using bound::Box;
using sensor::SphereSensorParams;

class BallsInBoxOccupancyMap
    : public OccupancyMap<fastrack_msgs::SensedSpheres, SphereSensorParams> {
 public:
  ~BallsInBoxOccupancyMap() {}
  explicit BallsInBoxOccupancyMap()
      : OccupancyMap<fastrack_msgs::SensedSpheres, SphereSensorParams>(),
        largest_obstacle_radius_(0.0),
        largest_sensor_radius_(0.0) {}

  // Derived classes must provide an OccupancyProbability function for both
  // single points and tracking error bounds centered on a point.
  // Ignores time since this is a time-invariant environment.
  double OccupancyProbability(
      const Vector3d& p,
      double time = std::numeric_limits<double>::quiet_NaN()) const;
  double OccupancyProbability(
      const Vector3d& p, const Box& bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const;
  double OccupancyProbability(
      const Vector3d& p, const Sphere& bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const;
  double OccupancyProbability(
      const Vector3d& p, const Cylinder& bound,
      double time = std::numeric_limits<double>::quiet_NaN()) const;

  // Generate a sensor measurement.
  fastrack_msgs::SensedSpheres SimulateSensor(
      const SphereSensorParams& params) const;

  // Derived classes must have some sort of visualization through RViz.
  void Visualize() const;

 private:
  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via OccupancyMap::LoadParameters).
  bool LoadParameters(const ros::NodeHandle& n);

  // Update this environment with the information contained in the given
  // sensor measurement.
  // NOTE! This function needs to publish on `updated_topic_`.
  void SensorCallback(
      const typename fastrack_msgs::SensedSpheres::ConstPtr& msg);

  // KdtreeMaps to store spherical obstacle and sensor locations, as well as
  // radii for each.
  KdtreeMap<3, double> obstacles_;
  KdtreeMap<3, double> sensor_fovs_;

  // Remember the largest obstacle/sensor radius yet, for intersection checks.
  double largest_obstacle_radius_;
  double largest_sensor_radius_;

  // Static constants for occupied/unknown/free probabilities.
  static constexpr double kOccupiedProbability = 1.0;
  static constexpr double kUnknownProbability = 0.5;
  static constexpr double kFreeProbability = 0.0;
};  //\class BallsInBoxOccupancyMap

}  //\namespace environment
}  //\namespace fastrack

#endif
