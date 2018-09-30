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
// LQR controller for the Crazyflie. Uses an LQR control matrix for hovering
// at each specified reference point.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_lqr/linear_feedback_controller.h>

namespace crazyflie_lqr {

// Initialize.
bool LinearFeedbackController::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "crazyflie_lqr");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Load K, x_ref, u_ref from disk.
  if (!LoadFromDisk()) {
    ROS_ERROR("%s: Failed to load K, x_ref, u_ref from disk.", name_.c_str());
    return false;
  }

  // Set up control publisher.
  ros::NodeHandle nl(n);
  control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    control_topic_.c_str(), 1, false);

  initialized_ = true;
  return true;
}

// Load K, x_ref, u_ref from disk.
bool LinearFeedbackController::LoadFromDisk() {
  // Set up file io to read K, x_ref, and u_ref from disk.
  K_ = MatrixXd::Zero(u_dim_, x_dim_);
  x_ref_ = VectorXd::Zero(x_dim_);
  u_ref_ = VectorXd::Zero(u_dim_);

  std::ifstream K_file(K_filename_);
  std::ifstream u_ref_file(u_ref_filename_);

  // Read K.
  if (K_file.is_open()) {
    for (size_t ii = 0; ii < u_dim_ && K_file.good(); ii++)
      for (size_t jj = 0; jj < x_dim_ && K_file.good(); jj++)
        K_file >> K_(ii, jj);
  } else {
    ROS_ERROR("%s: Could not find %s.", name_.c_str(), K_filename_.c_str());
    return false;
  }

  // Read u_ref.
  if (u_ref_file.is_open()) {
    for (size_t ii = 0; ii < u_dim_ && u_ref_file.good(); ii++)
      u_ref_file >> u_ref_(ii);
  } else {
    ROS_ERROR("%s: Could not find %s.", name_.c_str(), u_ref_filename_.c_str());
    return false;
  }

  std::cout << "K is: \n" << K_ << std::endl;
  std::cout << "uref is: \n" << u_ref_.transpose() << std::endl;
  return true;
}

// Load parameters. This may be overridden by derived classes.
bool LinearFeedbackController::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Text files with K, x_ref, u_ref.
  if (!nl.getParam("K_file", K_filename_)) return false;
  if (!nl.getParam("u_ref_file", u_ref_filename_)) return false;

  // Dimensions.
  int dimension = 1;
  if (!nl.getParam("x_dim", dimension)) return false;
  x_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("u_dim", dimension)) return false;
  u_dim_ = static_cast<size_t>(dimension);

  // Topics.
  if (!nl.getParam("topics/state", state_topic_)) return false;
  if (!nl.getParam("topics/reference", reference_topic_)) return false;
  if (!nl.getParam("topics/control", control_topic_)) return false;

  return true;
}

// Compute LQR control given the current state.
VectorXd LinearFeedbackController::Control(const VectorXd& x) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!initialized_) {
    ROS_ERROR("LinearFeedbackController was not initialized.");
    return VectorXd::Zero(1);
  }

  if (x.size() != x_dim_) {
    ROS_ERROR("%s: State was the wrong dimension: %zu vs. %zu.",
              name_.c_str(), x.size(), x_dim_);
    return u_ref_;
  }
#endif

  if (!received_reference_) {
    ROS_WARN("%s: Tried to get control before sending reference.", name_.c_str());
    return VectorXd::Zero(u_dim_);
  }

  return K_ * (x - x_ref_) + u_ref_;
}

} //\namespace crazyflie_lqr
