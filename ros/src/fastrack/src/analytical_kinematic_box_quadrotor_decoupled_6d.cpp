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
// Defines the AnalyticalKinematicBoxQuadrotorDecoupled6D class, which inherits
// from the ValueFunction base class. This class assumes Kinematic planner
// dynamics and QuadrotorDecoupled6D tracker dynamics, and uses a Box tracking
// error bound.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/value/analytical_kinematic_box_quadrotor_decoupled_6d.h>

namespace fastrack {
namespace value {

// Load parameters.
bool AnalyticalKinematicBoxQuadrotorDecoupled6D::LoadParameters(
    const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Set dynamics parameters.
  QuadrotorControl qc_lower, qc_upper;
  qc_lower.yaw_rate = 0.0;
  qc_upper.yaw_rate = 0.0;

  if (!nl.getParam("tracker/upper/pitch", qc_upper.pitch)) return false;
  if (!nl.getParam("tracker/upper/roll", qc_upper.roll)) return false;
  if (!nl.getParam("tracker/upper/thrust", qc_upper.thrust)) return false;
  if (!nl.getParam("tracker/lower/thrust", qc_lower.thrust)) return false;
  qc_lower.pitch = -qc_upper.pitch;
  qc_lower.roll = -qc_upper.roll;

  tracker_dynamics_.Initialize(std::unique_ptr<QuadrotorControlBoundBox>(
      new QuadrotorControlBoundBox(qc_lower, qc_upper)));

  VectorXd max_planner_speed(3);
  if (!nl.getParam("planner/vx", max_planner_speed(0))) return false;
  if (!nl.getParam("planner/vy", max_planner_speed(1))) return false;
  if (!nl.getParam("planner/vz", max_planner_speed(2))) return false;

  planner_dynamics_.Initialize(std::unique_ptr<VectorBoundBox>(
      new VectorBoundBox(-max_planner_speed, max_planner_speed)));

  // Set relative dynamics.
  relative_dynamics_.reset(new QuadrotorDecoupled6DRelKinematics);

  // Compute maximum acceleration. Make sure all elements are positive.
  const auto x_dot_max = tracker_dynamics_.Evaluate(
      PositionVelocity(Vector3d::Zero(), Vector3d::Zero()), qc_upper);

  max_acc_ = x_dot_max.Velocity();
  max_acc_(0) = std::abs(max_acc_(0));
  max_acc_(1) = std::abs(max_acc_(1));
  max_acc_(2) = std::abs(max_acc_(2));

  // Velocity/acceleration disturbance bounds.
  if (!nl.getParam("disturbance/velocity/x", vel_dist_(0))) return false;
  if (!nl.getParam("disturbance/velocity/y", vel_dist_(1))) return false;
  if (!nl.getParam("disturbance/velocity/z", vel_dist_(2))) return false;
  if (!nl.getParam("disturbance/acceleration/x", acc_dist_(0))) return false;
  if (!nl.getParam("disturbance/acceleration/y", acc_dist_(1))) return false;
  if (!nl.getParam("disturbance/acceleration/z", acc_dist_(2))) return false;

  // Position/velocity expansion.
  if (!nl.getParam("expansion/velocity/x", vel_exp_(0))) return false;
  if (!nl.getParam("expansion/velocity/y", vel_exp_(1))) return false;
  if (!nl.getParam("expansion/velocity/z", vel_exp_(2))) return false;
  pos_exp_ = vel_exp_.cwiseProduct(2.0 * max_planner_speed + 0.5 * vel_exp_)
                 .cwiseQuotient(max_acc_ - acc_dist_);

  // Generate the tracking error bound.
  bound_.x = pos_exp_(0) + (max_planner_speed(0) + vel_dist_(0)) *
                               (max_planner_speed(0) + vel_dist_(0)) /
                               (max_acc_(0) - acc_dist_(0));
  bound_.y = pos_exp_(1) + (max_planner_speed(1) + vel_dist_(1)) *
                               (max_planner_speed(1) + vel_dist_(1)) /
                               (max_acc_(1) - acc_dist_(1));
  bound_.z = pos_exp_(2) + (max_planner_speed(2) + vel_dist_(2)) *
                               (max_planner_speed(2) + vel_dist_(2)) /
                               (max_acc_(2) - acc_dist_(2));

  return true;
}

// Register callbacks.
bool AnalyticalKinematicBoxQuadrotorDecoupled6D::RegisterCallbacks(
    const ros::NodeHandle& n) {
  return true;
}

// Evaluate the value function at tracker/planner states.
double AnalyticalKinematicBoxQuadrotorDecoupled6D::Value(
    const PositionVelocity& tracker_x,
    const PositionVelocity& planner_x) const {
  // Get relative state.
  const PositionVelocityRelPositionVelocity relative_x(tracker_x, planner_x);
  const Vector3d& rx_position = relative_x.State().Position();
  const Vector3d& rx_velocity = relative_x.State().Velocity();

  // Get the maximum allowable control in each subsystem.
  const auto& control_bound =
      static_cast<const VectorBoundBox&>(planner_dynamics_.GetControlBound());
  const auto& max_planner_u = control_bound.Max();

  // Value is the maximum of values in each 2D subsystem.
  double value = -std::numeric_limits<double>::infinity();
  for (size_t ii = 0; ii < 3; ii++) {
    const double x = rx_position(ii);
    const double v = rx_velocity(ii);
    const double v_p = max_planner_u(ii);

    // Value surface A: + for x "below" convex acceleration parabola.
    const double value_A = -x - pos_exp_(ii) +
                           (0.5 * (v - v_p) * (v - v_p) - v_p * v_p) /
                               (max_acc_(ii) - acc_dist_(ii));

    // Value surface B: + for x "above" concave braking parabola.
    const double value_B = x + pos_exp_(ii) -
                           (-0.5 * (v + v_p) * (v + v_p) + v_p * v_p) /
                               (max_acc_(ii) - acc_dist_(ii));

    // Value function is the maximum of the above two surfaces.
    value = std::max(value, std::max(value_A, value_B));
  }

  return value;
}

// Compute the value function gradient at a pair of tracker/planner states.s
PositionVelocityRelPositionVelocity
AnalyticalKinematicBoxQuadrotorDecoupled6D::Gradient(
    const PositionVelocity& tracker_x,
    const PositionVelocity& planner_x) const {
  // Get relative state.
  const PositionVelocityRelPositionVelocity relative_x(tracker_x, planner_x);
  const Vector3d& rx_position = relative_x.State().Position();
  const Vector3d& rx_velocity = relative_x.State().Velocity();

  // Get the maximum allowable control in each subsystem.
  const auto& control_bound =
      static_cast<const VectorBoundBox&>(planner_dynamics_.GetControlBound());
  const auto& max_planner_u = control_bound.Max();

  // Loop through each subsystem and populate grad in position/velocity dims.
  Vector3d pos_grad, vel_grad;
  for (size_t ii = 0; ii < 3; ii++) {
    const double x = rx_position(ii);
    const double v = rx_velocity(ii);
    const double v_p = max_planner_u(ii);

    // Value surface A: + for x "below" convex acceleration parabola.
    const double value_A = -x - pos_exp_(ii) +
                           (0.5 * (v - v_p) * (v - v_p) - v_p * v_p) /
                               (max_acc_(ii) - acc_dist_(ii));

    // Value surface B: + for x "above" concave braking parabola.
    const double value_B = x + pos_exp_(ii) -
                           (-0.5 * (v + v_p) * (v + v_p) + v_p * v_p) /
                               (max_acc_(ii) - acc_dist_(ii));

    // Check which side we're on. If on A side, grad points toward -pos,
    // if on B side, grad points toward +pos.
    if (value_A > value_B) {
      pos_grad(ii) = -1.0;
      vel_grad(ii) = (v - v_p) / (max_acc_(ii) - acc_dist_(ii));
    } else {
      pos_grad(ii) = 1.0;
      vel_grad(ii) = (v + v_p) / (max_acc_(ii) - acc_dist_(ii));
    }
  }

  return PositionVelocityRelPositionVelocity(pos_grad, vel_grad));
}

// Priority of the optimal control at the given vehicle and planner states.
// This is a number between 0 and 1, where 1 means the final control signal
// should be exactly the optimal control signal computed by this
// value function.
double AnalyticalKinematicBoxQuadrotorDecoupled6D::Priority(
    const PositionVelocity& tracker_x,
    const PositionVelocity& planner_x) const {
  // Get value at this relative state.
  const double value = Value(tracker_x, planner_x);

  // HACK! The threshold should probably be externally set via config.
  const double relative_high = 0.20;  // 20% of max inside value
  const double relative_low = 0.05;   // 5% of max inside value

  // TODO! Make sure this is actually the maximum value.
  const double max_value = std::max(bound_.x, std::max(bound_.y, bound_.z));
  const double value_high = relative_high * max_value;
  const double value_low = relative_low * max_value;

  const double priority =
      1.0 -
      std::max(0.0,
               std::min((value - value_low) / (value_high - value_low), 1.0));

  return priority;
}

}  // namespace value
}  // namespace fastrack
