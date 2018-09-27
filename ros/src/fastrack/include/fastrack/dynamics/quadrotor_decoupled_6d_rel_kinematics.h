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
// 3D kinematic point model. Templated on type of control bound for quadrotor.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_QUADROTOR_DECOUPLED_6D_REL_KINEMATICS_H
#define FASTRACK_DYNAMICS_QUADROTOR_DECOUPLED_6D_REL_KINEMATICS_H

#include <fastrack/control/quadrotor_control.h>
#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/kinematics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/state/position_velocity_rel_position_velocity.h>
#include <fastrack/state/relative_state.h>

#include <math.h>

namespace fastrack {
namespace dynamics {

using control::QuadrotorControl;
using dynamics::Kinematics;
using dynamics::QuadrotorDecoupled6D;
using state::PositionVelocity;
using state::PositionVelocityRelPositionVelocity;
using state::RelativeState;

template <typename CB>
class QuadrotorDecoupled6DRelKinematics
    : public RelativeDynamics<PositionVelocity, QuadrotorControl,
                              PositionVelocity, VectorXd> {
 public:
  ~QuadrotorDecoupled6DRelKinematics() {}
  explicit QuadrotorDecoupled6DRelKinematics()
      : RelativeDynamics<PositionVelocity, QuadrotorControl, PositionVelocity,
                         VectorXd>() {}

  // Derived classes must be able to give the time derivative of relative state
  // as a function of current state and control of each system.
  inline std::unique_ptr<RelativeState<PositionVelocity, PositionVelocity>>
  Evaluate(const PositionVelocity& tracker_x, const QuadrotorControl& tracker_u,
           const PositionVelocity& planner_x, const VectorXd& planner_u) const {
    if (planner_u.size() != PositionVelocity::ConfigurationDimension())
      std::runtime_error("Bad planner control size.");

    // TODO(@jaime): confirm that this works. I set things up so things like
    // this should work.
    const QuadrotorDecoupled6D<CB> quad_dynamics;
    const Kinematics<PositionVelocity> quad_kinematics;
    return std::unique_ptr<PositionVelocityRelPositionVelocity>(
        new PositionVelocityRelPositionVelocity(
            quad_dynamics.Evaluate(tracker_x, tracker_u),
            quad_kinematics.Evaluate(planner_x, planner_u)));
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the relative state specified
  // by the given system states, provided abstract control bounds.
  inline QuadrotorControl OptimalControl(
      const PositionVelocity& tracker_x, const PositionVelocity& planner_x,
      const RelativeState<PositionVelocity, PositionVelocity>& value_gradient,
      const ControlBound<QuadrotorControl>& tracker_u_bound,
      const ControlBound<VectorXd>& planner_u_bound) const {
    // Get internal state of value gradient and map tracker control (negative)
    // coefficients to QuadrotorControl, so we get a negative gradient.
    const auto& cast = static_cast<const PositionVelocityRelPositionVelocity& >(
        value_gradient);

    const auto& grad = cast.State();
    QuadrotorControl negative_grad;
    negative_grad.yaw_rate = 0.0;
    negative_grad.pitch = -grad.Vx();
    negative_grad.roll = grad.Vy();
    negative_grad.thrust = -grad.Vz();

    // Project onto tracker control bound and make sure to zero out yaw_rate.
    QuadrotorControl c = tracker_u_bound.ProjectToSurface(negative_grad);
    c.yaw_rate = 0.0;

    return c;
  }
}; //\class QuadrotorDecoupled6DRelKinematics

} // namespace dynamics
} // namespace fastrack

#endif
