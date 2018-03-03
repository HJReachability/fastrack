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

#ifndef FASTRACK_VALUE_ANALYTICAL_KINEMATIC_BOX_QUADROTOR_DECOUPLED_6D_H
#define FASTRACK_VALUE_ANLAYTICAL_KINEMATIC_BOX_QUADROTOR_DECOUPLED_6D_H

#include <fastrack/state/position_velocity.h>
#include <fastrack/control/quadrotor_control.h>
#include <fastrack/dynamics/quadrotor_decoupled_6d.h>
#include <fastrack/dynamics/kinematics.h>
#include <fastrack/bound/box.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>

namespace fastrack {
namespace value {

using control::QuadrotorControl;
using state::PositionVelocity;
using dynamics::QuadrotorDecoupled6D;
using dynamics::Kinematics;
using bound::Box;

class AnalyticalKinematicBoxQuadrotorDecoupled6D : public ValueFunction<
  PositionVelocity, QuadrotorControl, QuadrotorDecoupled6D,
  PositionVelocity, VectorXd, Kinematics<PositionVelocity>, Box> {
public:
  ~AnalyticalKinematicBoxQuadrotorDecoupled6D() {}
  explicit AnalyticalKinematicBoxQuadrotorDecoupled6D()
    : ValueFunction() {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

  // Value and gradient at particular relative states.
  double Value(const PositionVelocity& vehicle_x,
               const PositionVelocity& planner_x) const;
  PositionVelocity Gradient(const PositionVelocity& vehicle_x,
                            const PositionVelocity& planner_x) const;

  // Priority of the optimal control at the given vehicle and planner states.
  // This is a number between 0 and 1, where 1 means the final control signal
  // should be exactly the optimal control signal computed by this
  // value function.
  double Priority(const PositionVelocity& vehicle_x,
                  const PositionVelocity& planner_x) const;

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Maximum acceleration.
  Vector3d max_acc_;

  // Disturbance bounds (assumed to be symmetric).
  Vector3d vel_dist_;
  Vector3d acc_dist_;

  // Position/velocity expansion.
  Vector3d pos_exp_;
  Vector3d vel_exp_;
}; //\class AnalyticalKinematicBoxQuadrotorDecoupled6D

} //\namespace value
} //\namespace fastrack

#endif
