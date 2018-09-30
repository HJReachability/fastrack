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
// Near hover simulator.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_SIMULATOR_NEAR_HOVER_SIMULATOR_H
#define CRAZYFLIE_SIMULATOR_NEAR_HOVER_SIMULATOR_H

#include <crazyflie_simulator/near_hover_dynamics.h>
#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include <fstream>

namespace crazyflie_simulator {

class NearHoverSimulator {
public:
  ~NearHoverSimulator() {}
  NearHoverSimulator()
    : received_control_(false),
      initialized_(false) {}

  // Initialize this class by reading parameters and loading callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Timer callback.
  void TimerCallback(const ros::TimerEvent& e);

  // Update control signal.
  void ControlCallback(const crazyflie_msgs::ControlStamped::ConstPtr& msg);

  // Current state and control.
  VectorXd x_;
  VectorXd u_;
  NearHoverDynamics dynamics_;

  // Flag for whether first control signal has been received.
  bool received_control_;

  // Timer.
  ros::Timer timer_;
  double dt_;
  ros::Time last_time_;

  // TF broadcasting.
  tf2_ros::TransformBroadcaster br_;

  // Publishers and subscribers.
  ros::Subscriber control_sub_;
  std::string control_topic_;

  // Frames of reference.
  std::string fixed_frame_id_;
  std::string robot_frame_id_;

  // Initialized flag and name.
  bool initialized_;
  std::string name_;
}; //\class NearHoverSimulator

} //\namespace crazyflie_simulator

#endif
