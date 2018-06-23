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
// Defines the relative dynamics between a 6D decoupled quadrotor model and a
// 3D planar Dubins model.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_QUADROTOR_DECOUPLED_6D_REL_PLANAR_DUBINS_3D_H
#define FASTRACK_DYNAMICS_QUADROTOR_DECOUPLED_6D_REL_PLANAR_DUBINS_3D_H

#include <fastrack/control/quadrotor_control.h>
#include <fastrack/dynamics/dynamics.h>
#include <fastrack/state/planar_dubins_3d.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/state/position_velocity_rel_planar_dubins_3d.h>

#include <math.h>

namespace fastrack {
namespace dynamics {

using control::QuadrotorControl;
using state::PlanarDubins3D;
using state::PositionVelocity;
using state::PositionVelocityRelPlanarDubins3D;

class QuadrotorDecoupled6DRelPlanarDubins3D
    : public RelativeDynamics<PositionVelocity, QuadrotorControl,
                              PlanarDubins3D, double> {
public:
  ~QuadrotorDecoupled6DRelPlanarDubins3D() {}
  explicit QuadrotorDecoupled6DRelPlanarDubins3D()
      : RelativeDynamics<PositionVelocity, QuadrotorControl, PlanarDubins3D,
                         double>() {}

  // Derived classes must be able to give the time derivative of relative state
  // as a function of current state and control of each system.
  inline std::unique_ptr<RelativeState<PositionVelocity, PlanarDubins3D>>
  Evaluate(const PositionVelocity &tracker_x, const QuadrotorControl &tracker_u,
           const PlanarDubins3D &planner_x, const double &planner_u) const {
    // Compute relative state.
    const PositionVelocityRelPlanarDubins3D relative_x(tracker_x, planner_x);

    // TODO(@jaime): Check these calculations.
    // Relative distance derivative.
    distance_dot =
        relative_x.TangentVelocity() * std::cos(relative_x.Bearing()) +
        relative_x.NormalVelocity() * std::sin(relative_x.Bearing());

    // Relative bearing derivative.
    bearing_dot =
        -planner_u -
        relative_x.TangentVelocity() * std::sin(relative_x.Bearing()) +
        relative_x.NormalVelocity() * std::cos(relative_x.Bearing());

    // Relative tangent and normal velocity derivatives.
    // NOTE! Must rotate roll/pitch into planner frame.
    const double c = std::cos(planner_x.Theta());
    const double s = std::sin(planner_x.Theta());

    tangent_velocity_dot = tracker_u.pitch * c - tracker_u.roll * s +
                           planner_u * relative_x.TangentVelocity();
    normal_velocity_dot = -tracker_u.pitch * s - tracker_u.roll * c -
                          planner_u * relative_x.TangentVelocity();

    return std::unique_ptr<PositionVelocityRelPlanarDubins3D>(
        new PositionVelocityRelPlanarDubins3D() distance_dot, bearing_dot,
        tangent_velocity_dot, normal_velocity_dot));
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the relative state specified
  // by the given system states, provided abstract control bounds.
  inline QuadrotorControl OptimalControl(
      const PositionVelocity &tracker_x, const PlanarDubins3D &planner_x,
      const RelativeState<PositionVelocity, PlanarDubins3D> &value_gradient,
      const QuadrotorControlBoundCylinder &tracker_u_bound,
      const ScalarBoundInterval &planner_u_bound) const {
    // TODO(@dfk, @jaime): figure this part out.
  }
}; //\class QuadrotorDecoupledPlanarDubins

} // namespace dynamics
} // namespace fastrack

#endif
