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
// Defines the Quadrotor6D class, which uses PositionVelocity as the state
// and QuadrotorControl as the control.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_QUADROTOR_6D_H
#define FASTRACK_DYNAMICS_QUADROTOR_6D_H

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/control/quadrotor_control.h>

#include <math.h>

namespace fastrack {
namespace dynamics {

using state::PositionVelocity;
using control::QuadrotorControl;

class QuadrotorDecoupled6D :
    public Dynamics<PositionVelocity, QuadrotorControl> {
public:
  ~QuadrotorDecoupled6D() {}
  explicit QuadrotorDecoupled6D(const QuadrotorControl& u_lower,
                                const QuadrotorControl& u_upper)
    : Dynamics(u_lower, u_upper) {}

  // Derived classes must be able to give the time derivative of state
  // as a function of current state and control.
  inline PositionVelocity Evaluate(
    const PositionVelocity& x, const QuadrotorControl& u) const {
    // Position derivatives are just velocity.
    const Vector3d position_dot(x.Velocity());

    // Velocity derivatives are given by simple trigonometric functions
    // of the pitch/roll, scaled by G since we assume that thrust is
    // approximately equal to G.
    const Vector3d velocity_dot(constants::G * std::tan(u.pitch),
                                -constants::G * std::tan(u.roll),
                                u.thrust - constants::G);

    return PositionVelocity(position_dot, velocity_dot);
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the specified state.
  // In this case (linear dynamics), the state is irrelevant given the
  // gradient of the value function at that state.
  inline QuadrotorControl OptimalControl(
    const PositionVelocity& x, const PositionVelocity& value_gradient) const {
    // Set each dimension of optimal control to upper/lower bound depending
    // on the sign of the gradient in that dimension. We want to minimize the
    // inner product between the projected gradient and control.
    // If the gradient is 0, then sets control to zero by default.
    QuadrotorControl c;
    c.yaw_rate = 0.0;
    c.pitch = (value_gradient.Vx() < 0.0) ? u_upper_.pitch : u_lower_.pitch;
    c.roll = (value_gradient.Vy() > 0.0) ? u_upper_.roll : u_lower_.roll;
    c.thrust = (value_gradient.Vz() < 0.0) ? u_upper_.thrust : u_lower_.thrust;

    return c;
  }
}; //\class Quadrotor6D

} //\namespace dynamics
} //\namespace fastrack

#endif
