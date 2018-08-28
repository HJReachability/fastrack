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
 *          Jaime F. Fisac         ( jfisac@eecs.berkeley.edu )
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
#include <fastrack/control/quadrotor_control_bound_cylinder.h>
#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/planar_dubins_3d.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/state/position_velocity_rel_planar_dubins_3d.h>

#include <math.h>

namespace fastrack {
namespace dynamics {

using control::QuadrotorControl;
using control::QuadrotorControlBoundCylinder;
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
  Evaluate(const PositionVelocity& tracker_x, const QuadrotorControl& tracker_u,
           const PlanarDubins3D& planner_x, const double& planner_u) const {
    // Compute relative state.
    const PositionVelocityRelPlanarDubins3D relative_x(tracker_x, planner_x);

    // Net instantaneous tangent velocity (PositionVelocity minus Dubins).
    // This is used in the derivatives of relative position (distance, bearing).
    // It is NOT used in the velocity derivatives because velocity states are
    // absolute (even though they are expressed in the changing Dubins frame).
    const auto net_tangent_velocity =
        relative_x.TangentVelocity() - planner_x.V();

    // Relative distance derivative.
    const double distance_dot =
        net_tangent_velocity * std::cos(relative_x.Bearing()) +
        relative_x.NormalVelocity() * std::sin(relative_x.Bearing());

    // Relative bearing derivative.
    const double bearing_dot =
        -planner_u +
        (-net_tangent_velocity * std::sin(relative_x.Bearing()) +
          relative_x.NormalVelocity() * std::cos(relative_x.Bearing())) /
        relative_x.Distance(); // omega_circ = v_circ / R

    // Tracker accelerations expressed in inertial world frame.
    const double tracker_accel_x = constants::G * std::tan(tracker_u.pitch);
    const double tracker_accel_y = -constants::G * std::tan(tracker_u.roll);

    // Relative tangent and normal velocity derivatives.
    // NOTE! Must rotate roll/pitch into planner frame.
    const double c = std::cos(planner_x.Theta());
    const double s = std::sin(planner_x.Theta());

    const double tangent_velocity_dot =
        tracker_accel_x * c + tracker_accel_y * s +
        planner_u * relative_x.NormalVelocity();
    const double normal_velocity_dot =
        -tracker_accel_x * s + tracker_accel_y * c -
        planner_u * relative_x.TangentVelocity();

    return std::unique_ptr<PositionVelocityRelPlanarDubins3D>(
        new PositionVelocityRelPlanarDubins3D( distance_dot, bearing_dot,
        tangent_velocity_dot, normal_velocity_dot));
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the relative state specified
  // by the given system states, provided abstract control bounds.
  inline QuadrotorControl OptimalControl(
      const PositionVelocity& tracker_x, const PlanarDubins3D& planner_x,
      const RelativeState<PositionVelocity, PlanarDubins3D>& value_gradient,
      const ControlBound<QuadrotorControl>& tracker_u_bound,
      const ControlBound<double>& planner_u_bound) const {
    // TODO(@dfk, @jaime): figure this part out.
    return QuadrotorControl();
  }
}; //\class QuadrotorDecoupledPlanarDubins

} // namespace dynamics
} // namespace fastrack

#endif
