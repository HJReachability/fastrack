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
 * Authors: David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 *          Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// LQR hover controller for the Crazyflie. Assumes that the state space is
// given by the PositionVelocityYawStateStamped message type, which is a 7D model but the
// reference is only a 6D PositionStateStamped message type (appends zero yaw).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_LQR_POSITION_VELOCITY_YAW_STATE_LIFT_LQR_H
#define CRAZYFLIE_LQR_POSITION_VELOCITY_YAW_STATE_LIFT_LQR_H

#include <crazyflie_lqr/linear_feedback_controller.h>
#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/PositionVelocityYawStateStamped.h>
#include <crazyflie_msgs/PositionVelocityStateStamped.h>

#include <ros/ros.h>
#include <math.h>
#include <fstream>

namespace crazyflie_lqr {

class PositionVelocityYawStateLiftLqr : public LinearFeedbackController {
public:
  virtual ~PositionVelocityYawStateLiftLqr() {}
  explicit PositionVelocityYawStateLiftLqr()
    : LinearFeedbackController(),
      x_int_(Vector3d::Zero()),
      x_int_thresh_(Vector3d::Zero()) {}

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Process an incoming reference point.
  void ReferenceCallback(
    const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Process an incoming state measurement.
  void StateCallback(
    const crazyflie_msgs::PositionVelocityYawStateStamped::ConstPtr& msg);

  // Integral of position error.
  Vector3d x_int_;
  Vector3d x_int_thresh_;
  Vector3d integrator_k_;
}; //\class PositionVelocityYawStateLiftLqr

} //\namespace crazyflie_lqr

#endif
