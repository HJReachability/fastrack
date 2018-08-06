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
// Cylindrical bound for quadrotor controls. Circle in (pitch, roll), and
// intervals in yaw_rate and thrust.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CONTROL_QUADROTOR_CONTROL_BOUND_CYLINDER_H
#define FASTRACK_CONTROL_QUADROTOR_CONTROL_BOUND_CYLINDER_H

#include <fastrack/control/control_bound.h>
#include <fastrack/control/quadrotor_control.h>
#include <fastrack/control/scalar_bound_interval.h>

#include <math.h>

namespace fastrack {
namespace control {

class QuadrotorControlBoundCylinder : public ControlBound<QuadrotorControl> {
 public:
  ~QuadrotorControlBoundCylinder() {}
  explicit QuadrotorControlBoundCylinder(double radius,
                                         double ScalarBoundInterval &yaw_rate,
                                         double ScalarBoundInterval &thrust)
      : pitch_roll_radius_(radius),
        yaw_rate_interval_(yaw_rate),
        thrust_interval_(thrust) {}

  // Assume 'params' is laid out as follows:
  // [radius, min yaw rate, min thrust, max yaw rate, max thrust]
  explicit QuadrotorControlBoundCylinder(const std::vector<double> &params)
      : pitch_roll_radius_(params[0]),
        yaw_rate_interval_(params[1], params[3]),
        thrust_interval_(params[2], params[4]) {}

  // Derived classes must be able to check whether a query is inside the
  // bound.
  inline bool Contains(const QuadrotorControl &query) const {
    return std::hypot(query.pitch, query.roll) < pitch_roll_radius_ &&
           yaw_rate_interval_.Contains(query.yaw_rate) &&
           thrust_interval_.Contains(query.thrust);
  }

  // Derived classes must be able to compute the projection of a vector
  // (represented as the templated type) onto the surface of the bound.
  // NOTE: We will treat this vector as emanating from the natural origin
  // of the bound so that it constitutes a meaningful direction with respect
  // to that origin.
  inline QuadrotorControl ProjectToSurface(
      const QuadrotorControl &query) const {
    // Compute scaling to project (pitch, roll) onto the cylinder.
    const double scaling =
        pitch_roll_radius_ / std::hypot(query.pitch, query.roll);

    return QuadrotorControl(scaling * query.pitch, scaling * query.roll,
                            yaw_rate_interval_.ProjectToSurface(query.yaw_rate),
                            thrust_interval_.ProjectToSurface(query.thrust));
  }

 private:
  // Radius in (pitch, roll) dimensions.
  const double pitch_roll_radius_;

  // Intervals in yaw_rate and thrust.
  const ScalarBoundInterval yaw_rate_interval_;
  const ScalarBoundInterval thrust_interval_;
};  //\class ControlBound

}  // namespace control
}  // namespace fastrack

#endif
