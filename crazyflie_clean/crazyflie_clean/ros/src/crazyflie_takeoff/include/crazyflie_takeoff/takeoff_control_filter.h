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
// Class to filter control messages coming from the takeoff server from other
// sources of control signals so that the takeoff server takes precedence.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_TAKEOFF_TAKEOFF_CONTROL_FILTER_H
#define CRAZYFLIE_TAKEOFF_TAKEOFF_CONTROL_FILTER_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/ControlStamped.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace crazyflie_takeoff {

class TakeoffControlFilter {
public:
  ~TakeoffControlFilter() {}
  explicit TakeoffControlFilter()
    : in_flight_(false),
      received_takeoff_control_(false),
      initialized_(false) {}

  // Initialize this class.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for takeoff server controls.
  void TakeoffControlCallback(
    const crazyflie_msgs::ControlStamped::ConstPtr& msg);

  // Callback for commanded controls.
  void CommandedControlCallback(
    const crazyflie_msgs::ControlStamped::ConstPtr& msg);

  // In flight signal callback.
  void InFlightCallback(const std_msgs::Empty::ConstPtr& msg);

  // Most recent takeoff control.
  crazyflie_msgs::ControlStamped takeoff_control_;
  bool received_takeoff_control_;

  // Publishers, subscribers, and topics.
  ros::Publisher final_control_pub_;
  ros::Subscriber takeoff_control_sub_;
  ros::Subscriber commanded_control_sub_;
  ros::Subscriber in_flight_sub_;

  std::string final_control_topic_;
  std::string takeoff_control_topic_;
  std::string commanded_control_topic_;
  std::string in_flight_topic_;

  // Are we currently in flight?
  bool in_flight_;

  // Naming and initialization.
  bool initialized_;
  std::string name_;
}; //\class TakeoffControlFilter

} //\crazyflie_takeoff

#endif
