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
///
// State estimator node. Sets a recurring timer and every time it rings,
// this node will query tf for the transform between the specified robot
// frame and the fixed frame, merge it with the previous state estimate, and
// publish the new estimate.
//
// Derived classes specialize depending on the specific state space.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_STATE_ESTIMATOR_STATE_ESTIMATOR_H
#define CRAZYFLIE_STATE_ESTIMATOR_STATE_ESTIMATOR_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>
#include <math.h>

class StateEstimator {
public:
  virtual ~StateEstimator() {}

  // Initialize this class by reading parameters and loading callbacks.
  bool Initialize(const ros::NodeHandle& n);

protected:
  explicit StateEstimator()
    : tf_listener_(tf_buffer_),
      initialized_(false),
      first_update_(true) {}

  // Load parameters and register callbacks. These may/must be overridden
  // by derived classes.
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n) = 0;

  // Merge a pose measured at the given time (specified by translation
  // and euler angles) into the current state estimate.
  virtual void Update(const Vector3d& translation, const Vector3d& euler,
                      const ros::Time& stamp) = 0;

  // Whenever timer rings, query tf, update state estimate, and publish.
  void TimerCallback(const ros::TimerEvent& e);

  // Running state estimate.
  VectorXd x_;
  size_t x_dim_;

  // State publisher.
  ros::Publisher state_pub_;
  std::string state_topic_;

  // Frames of reference.
  std::string fixed_frame_id_;
  std::string robot_frame_id_;

  // Set a recurring timer for a discrete-time controller. Also keep track
  // of the most recent time when the timer fired.
  ros::Time last_time_;
  ros::Timer timer_;
  double dt_;

  // Buffer and listener to get current pose.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Name, and flags for initialization and first state update.
  bool initialized_;
  bool first_update_;
  std::string name_;
}; //\class StateEstimator

#endif
