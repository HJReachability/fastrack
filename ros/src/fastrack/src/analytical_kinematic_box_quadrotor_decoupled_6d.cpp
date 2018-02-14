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

// Initialize from a ROS NodeHandle.
bool AnalyticalKinematicBoxQuadrotorDecoupled6D::
Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(
    n.getNamespace(), "AnalyticalKinematicBoxQuadrotorDecoupled6D");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool AnalyticalKinematicBoxQuadrotorDecoupled6D::
LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Generate the tracking error bound.
  if (!nl.getParam("bound/x", bound_.x)) return false;
  if (!nl.getParam("bound/y", bound_.y)) return false;
  if (!nl.getParam("bound/z", bound_.z)) return false;

  // Set dynamics parameters.
  QuadrotorControl qc_lower, qc_upper;
  qc_lower.yaw_rate = 0.0;
  qc_upper.yaw_rate = 0.0;

  if (!nl.getParam("tracker/lower/pitch", qc_lower.pitch)) return false;
  if (!nl.getParam("tracker/lower/roll", qc_lower.roll)) return false;
  if (!nl.getParam("tracker/lower/thrust", qc_lower.thrust)) return false;
  if (!nl.getParam("tracker/upper/pitch", qc_upper.pitch)) return false;
  if (!nl.getParam("tracker/upper/roll", qc_upper.roll)) return false;
  if (!nl.getParam("tracker/upper/thrust", qc_upper.thrust)) return false;
  tracker_dynamics_.Initialize(qc_lower, qc_upper);

  VectorXd pc_lower(3), pc_upper(3);
  if (!nl.getParam("planner/lower/vx", pc_lower(0))) return false;
  if (!nl.getParam("planner/lower/vy", pc_lower(1))) return false;
  if (!nl.getParam("planner/lower/vz", pc_lower(2))) return false;
  if (!nl.getParam("planner/upper/vx", pc_upper(0))) return false;
  if (!nl.getParam("planner/upper/vy", pc_upper(1))) return false;
  if (!nl.getParam("planner/upper/vz", pc_upper(2))) return false;
  planner_dynamics_.Initialize(pc_lower, pc_upper);
}

// Register callbacks.
bool AnalyticalKinematicBoxQuadrotorDecoupled6D::
RegisterCallbacks(const ros::NodeHandle& n) { return true; }

// Evaluate the value function at tracker/planner states.
double AnalyticalKinematicBoxQuadrotorDecoupled6D::
Value(const PositionVelocity& tracker_x,
      const PositionVelocity& planner_x) const {}

// Compute the value function gradient at a pair of tracker/planner states.
PositionVelocity AnalyticalKinematicBoxQuadrotorDecoupled6D::
Gradient(const PositionVelocity& tracker_x,
         const PositionVelocity& planner_x) const {}

// Priority of the optimal control at the given vehicle and planner states.
// This is a number between 0 and 1, where 1 means the final control signal
// should be exactly the optimal control signal computed by this
// value function.
double AnalyticalKinematicBoxQuadrotorDecoupled6D::
Priority(const PositionVelocity& tracker_x,
         const PositionVelocity& planner_x) const {}

} //\namespace value
} //\namespace fastrack
