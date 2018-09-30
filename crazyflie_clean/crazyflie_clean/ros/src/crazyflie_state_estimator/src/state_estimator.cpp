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
// State estimator node. Sets a recurring timer and every time it rings,
// this node will query tf for the transform between the specified robot
// frame and the fixed frame, merge it with the previous state estimate, and
// publish the new estimate.
//
// Derived classes specialize depending on the specific state space.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_state_estimator/state_estimator.h>

// Initialize this class by reading parameters and loading callbacks.
bool StateEstimator::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "state_estimator");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize state vector to zero.
  x_ = VectorXd::Zero(x_dim_);

  // Timer.
  ros::NodeHandle nl(n);
  timer_ = nl.createTimer(
    ros::Duration(dt_), &StateEstimator::TimerCallback, this);

  // Sleep for a little while to let other nodes start up.
  ros::Duration(0.25).sleep();

  initialized_ = true;
  return true;
}

// Load parameters.
bool StateEstimator::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // State dimension.
  int dimension = 1;
  if (!nl.getParam("x_dim", dimension)) return false;
  x_dim_ = static_cast<size_t>(dimension);

  // State topic.
  if (!nl.getParam("topics/state", state_topic_)) return false;

  // Frames of reference.
  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/robot", robot_frame_id_)) return false;

  // Time step for reading tf.
  if (!nl.getParam("time_step", dt_)) return false;

  return true;
}

// Whenever timer rings, query tf, update state estimate, and publish.
void StateEstimator::TimerCallback(const ros::TimerEvent& e) {
  const ros::Time right_now = ros::Time::now();

  // Get the current transform from tf.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), robot_frame_id_.c_str(), ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    return;
  }

  // Extract translation.
  const Vector3d translation(tf.transform.translation.x,
                             tf.transform.translation.y,
                             tf.transform.translation.z);

  // Get roll, pitch, and yaw from quaternion.
  const Quaterniond quat(tf.transform.rotation.w,
                         tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z);

  // Multiply by sign of x component to ensure quaternion giving the preferred
  // Euler transformation (here we're exploiting the fact that rot(q)=rot(-q) ).
  Eigen::Matrix3d R = quat.toRotationMatrix();
  Vector3d euler = crazyflie_utils::angles::Matrix2RPY(R);
  euler(0) = crazyflie_utils::angles::WrapAngleRadians(euler(0));
  euler(1) = crazyflie_utils::angles::WrapAngleRadians(euler(1));
  euler(2) = crazyflie_utils::angles::WrapAngleRadians(euler(2));

  // Update state.
  Update(translation, euler, right_now);

  // Update the time.
  last_time_ = right_now;
}
