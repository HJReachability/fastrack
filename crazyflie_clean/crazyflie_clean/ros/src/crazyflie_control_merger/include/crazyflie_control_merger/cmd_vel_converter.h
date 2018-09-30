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
// Class to convert ControlStamped messages to Twists and publish on cmd_vel.
// Provides services "takeoff" and "land" that determine whether cmd_vel gets
// 0 or the actual control signal requested.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_CONTROL_MERGER_CMD_VEL_CONVERTER_H
#define CRAZYFLIE_CONTROL_MERGER_CMD_VEL_CONVERTER_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_utils/pwm.h>
#include <crazyflie_msgs/ControlStamped.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <math.h>

namespace crazyflie_control_merger {

class CmdVelConverter {
public:
  ~CmdVelConverter() {}
  explicit CmdVelConverter()
    : initialized_(false) {}

  // Initialize this class.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Process an incoming reference point.
  void ControlCallback(const crazyflie_msgs::ControlStamped::ConstPtr& msg);

  // Publishers, subscribers, and topics.
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber control_sub_;

  std::string cmd_vel_topic_;
  std::string control_topic_;

  // Naming and initialization.
  bool initialized_;
  std::string name_;
}; //\class NoYawMerger

} //\crazyflie_control_merger

#endif
