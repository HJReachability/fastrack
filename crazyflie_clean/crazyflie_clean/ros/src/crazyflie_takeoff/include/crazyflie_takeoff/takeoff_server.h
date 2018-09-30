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
// Class to provide takeoff service.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_TAKEOFF_TAKEOFF_SERVER_H
#define CRAZYFLIE_TAKEOFF_TAKEOFF_SERVER_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/ControlStamped.h>
#include <crazyflie_msgs/Control.h>
#include <crazyflie_msgs/PositionVelocityStateStamped.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <math.h>

namespace crazyflie_takeoff {

class TakeoffServer {
public:
  ~TakeoffServer() {}
  explicit TakeoffServer()
    : in_flight_(false),
      initialized_(false) {}

  // Initialize this class.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Takeoff service. Set in_flight_ flag to true.
  bool TakeoffService(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res);

  // Takeoff service. Set in_flight_ flag to true.
  bool LandService(std_srvs::Empty::Request& req,
                   std_srvs::Empty::Response& res);

  // Timer callback for refreshing landing control signal.
  void TimerCallback(const ros::TimerEvent& e);

  // Publishers, subscribers, and topics.
  ros::Publisher control_pub_;
  ros::Publisher in_flight_pub_;
  ros::Publisher reference_pub_;

  std::string control_topic_;
  std::string in_flight_topic_;
  std::string reference_topic_;

  // Takeoff and landing services.
  std::string takeoff_srv_name_;
  ros::ServiceServer takeoff_srv_;

  std::string land_srv_name_;
  ros::ServiceServer land_srv_;

  // In flight flag.
  bool in_flight_;

  // Takeoff sequence params.
  double open_loop_duration_;
  double hover_duration_;

  // Timer for refreshing landing control signal.
  ros::Timer timer_;

  // Initial hover point.
  Vector3d hover_point_;

  // Naming and initialization.
  bool initialized_;
  std::string name_;
}; //\class TakeoffServer

} //\crazyflie_takeoff

#endif
