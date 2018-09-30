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
// Linear feedback controller that reads in control/references from files.
// Controllers for specific state spaces will be derived from this class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_LQR_LINEAR_FEEDBACK_CONTROLLER_H
#define CRAZYFLIE_LQR_LINEAR_FEEDBACK_CONTROLLER_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/ControlStamped.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <fstream>

namespace crazyflie_lqr {

class LinearFeedbackController {
public:
  virtual ~LinearFeedbackController() {}

  // Initialize this class by reading parameters and loading callbacks.
  virtual bool Initialize(const ros::NodeHandle& n);

  // Compute control given the current state.
  virtual VectorXd Control(const VectorXd& x) const;

protected:
  explicit LinearFeedbackController()
    : received_reference_(false),
      last_state_time_(-1.0),
      initialized_(false) {}

  // Load parameters and register callbacks. These may/must be overridden
  // by derived classes.
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n) = 0;

  // K matrix and reference state/control (to fight gravity). These are
  // hard-coded since they will not change.
  MatrixXd K_;
  VectorXd u_ref_;
  VectorXd x_ref_;

  std::string K_filename_;
  std::string u_ref_filename_;

  // Dimensions of control and state spaces.
  size_t x_dim_;
  size_t u_dim_;

  // Remember last time we got a state callback.
  double last_state_time_;

  // Publishers and subscribers.
  ros::Subscriber state_sub_;
  ros::Subscriber reference_sub_;
  ros::Publisher control_pub_;

  std::string state_topic_;
  std::string reference_topic_;
  std::string control_topic_;

  // Initialized flag and name.
  bool received_reference_;
  bool initialized_;
  std::string name_;

private:
  // Load K, x_ref, u_ref from disk.
  bool LoadFromDisk();
}; //\class LinearFeedbackController

} //\namespace crazyflie_lqr

#endif
