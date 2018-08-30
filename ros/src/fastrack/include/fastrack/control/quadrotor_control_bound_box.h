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
// Box bound for quadrotor controls.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CONTROL_QUADROTOR_CONTROL_BOUND_BOX_H
#define FASTRACK_CONTROL_QUADROTOR_CONTROL_BOUND_BOX_H

#include <fastrack/control/control_bound.h>
#include <fastrack/control/quadrotor_control.h>
#include <fastrack/control/scalar_bound_interval.h>

#include <math.h>

namespace fastrack {
namespace control {

class QuadrotorControlBoundBox : public ControlBound<QuadrotorControl> {
 public:
  ~QuadrotorControlBoundBox() {}
  explicit QuadrotorControlBoundBox(const QuadrotorControl& min,
                                    const QuadrotorControl& max)
      : pitch_interval_(min.pitch, max.pitch),
        roll_interval_(min.roll, max.roll),
        yaw_rate_interval_(min.yaw_rate, max.yaw_rate),
        thrust_interval_(min.thrust, max.thrust) {}

  // Assume params are laid out as follows:
  // [min pitch, min roll, min yaw rate, min thrust,
  //  max pitch, max roll, max yaw rate, max thrust]
  explicit QuadrotorControlBoundBox(const std::vector<double>& params)
      : pitch_interval_(params[0], params[4]),
        roll_interval_(params[1], params[5]),
        yaw_rate_interval_(params[2], params[6]),
        thrust_interval_(params[3], params[7]) {}

  // Custom definition of copy-assign operator.
  QuadrotorControlBoundBox& operator=(const QuadrotorControlBoundBox& other) {
    if (&other == this) return *this;

    pitch_interval_ = other.pitch_interval_;
    roll_interval_ = other.roll_interval_;
    yaw_rate_interval_ = other.yaw_rate_interval_;
    thrust_interval_ = other.thrust_interval_;
    return *this;
  }

  // Derived classes must be able to check whether a query is inside the
  // bound.
  inline bool Contains(const QuadrotorControl& query) const {
    return pitch_interval_.Contains(query.pitch) &&
           roll_interval_.Contains(query.roll) &&
           yaw_rate_interval_.Contains(query.yaw_rate) &&
           thrust_interval_.Contains(query.thrust);
  }

  // Derived classes must be able to compute the projection of a vector
  // (represented as the templated type) onto the surface of the bound.
  // NOTE: this is basically solving an LP with the bound as the feasible
  // set and the query as the coefficients.
  inline QuadrotorControl ProjectToSurface(
      const QuadrotorControl& query) const {
    return QuadrotorControl(pitch_interval_.ProjectToSurface(query.pitch),
                            roll_interval_.ProjectToSurface(query.roll),
                            yaw_rate_interval_.ProjectToSurface(query.yaw_rate),
                            thrust_interval_.ProjectToSurface(query.thrust));
  }

 private:
  // ScalarBoundIntervals for each control variable.
  ScalarBoundInterval pitch_interval_;
  ScalarBoundInterval roll_interval_;
  ScalarBoundInterval yaw_rate_interval_;
  ScalarBoundInterval thrust_interval_;
};  //\class ControlBound

}  // namespace control
}  // namespace fastrack

#endif
