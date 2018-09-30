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
// Class to handle takeoff and landing services.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_takeoff/takeoff_server.h>

namespace crazyflie_takeoff {

// Initialize this node.
bool TakeoffServer::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "takeoff_server");

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
bool TakeoffServer::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topics/reference", reference_topic_)) return false;

  // Hover point.
  double init_x, init_y, init_z;
  if (!nl.getParam("hover/x", init_x)) return false;
  if (!nl.getParam("hover/y", init_y)) return false;
  if (!nl.getParam("hover/z", init_z)) return false;
  hover_point_ = Vector3d(init_x, init_y, init_z);

  // Takeoff sequence params.
  if (!nl.getParam("duration/open_loop", open_loop_duration_))
    open_loop_duration_ = 1.0;
  if (!nl.getParam("duration/hover", hover_duration_))
    hover_duration_ = 5.0;

  // Service names.
  if (!nl.getParam("srv/takeoff", takeoff_srv_name_))
    takeoff_srv_name_ = "/takeoff";
  if (!nl.getParam("srv/land", land_srv_name_))
    land_srv_name_ = "/land";

  return true;
}

// Register callbacks.
bool TakeoffServer::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    control_topic_.c_str(), 10, false);

  reference_pub_ = nl.advertise<crazyflie_msgs::PositionVelocityStateStamped>(
    reference_topic_.c_str(), 10, false);

  in_flight_pub_ = nl.advertise<std_msgs::Empty>(
    in_flight_topic_.c_str(), 10, false);

  // Services.
  takeoff_srv_ = nl.advertiseService(takeoff_srv_name_,
                                     &TakeoffServer::TakeoffService, this);

  land_srv_ =
      nl.advertiseService(land_srv_name_, &TakeoffServer::LandService, this);

  // Timer.
  timer_ =
      nl.createTimer(ros::Duration(0.1), &TakeoffServer::TimerCallback, this);

  return true;
}

// Timer callback for refreshing landing control signal.
void TakeoffServer::TimerCallback(const ros::TimerEvent& e) {
  // If in flight, then no need to resend landing signal.
  if (in_flight_)
    return;

  // Send a zero thrust signal.
  crazyflie_msgs::ControlStamped msg;
  msg.header.stamp = ros::Time::now();

  msg.control.roll = 0.0;
  msg.control.pitch = 0.0;
  msg.control.yaw_dot = 0.0;
  msg.control.thrust = 0.0;

  control_pub_.publish(msg);
}

// Landing service.
bool TakeoffServer::
LandService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ROS_INFO("%s: Landing requested.", name_.c_str());

  // Let other nodes know that we are not in flight anymore.
  in_flight_pub_.publish(std_msgs::Empty());

  // Slowly spin the rotors down.
  const ros::Time right_now = ros::Time::now();
  while ((ros::Time::now() - right_now).toSec() < 1.0) {
    crazyflie_msgs::ControlStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.control.roll = 0.0;
    msg.control.pitch = 0.0;
    msg.control.yaw_dot = 0.0;

    // Slowly decrement thrust.
    msg.control.thrust = std::max(0.0, crazyflie_utils::constants::G -
                                  5.0 * (ros::Time::now() - right_now).toSec());

    control_pub_.publish(msg);

    // Sleep a little, then rerun the loop.
    ros::Duration(0.01).sleep();
  }

  in_flight_ = false;

  // Return true.
  return true;
}

// Takeoff service. Set in_flight_ flag to true.
bool TakeoffServer::
TakeoffService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  if (in_flight_) {
    ROS_WARN("%s: Tried to takeoff while in flight.", name_.c_str());
    return false;
  }

  ROS_INFO("%s: Takeoff requested.", name_.c_str());

  // Lift off, and after a short wait return.
  const ros::Time takeoff_time = ros::Time::now();
  while ((ros::Time::now() - takeoff_time).toSec() < open_loop_duration_) {
    crazyflie_msgs::ControlStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.control.roll = 0.0;
    msg.control.pitch = 0.0;
    msg.control.yaw_dot = 0.0;

    // Offset gravity, plus a little extra to lift off.
    msg.control.thrust = crazyflie_utils::constants::G + 0.2;
    control_pub_.publish(msg);

    // Sleep a little, then rerun the loop.
    ros::Duration(0.01).sleep();
  }

  // Send reference to LQR (which is hopefully running...).
  crazyflie_msgs::PositionVelocityStateStamped reference;
  reference.header.stamp = ros::Time::now();
  reference.state.x = hover_point_(0);
  reference.state.y = hover_point_(1);
  reference.state.z = hover_point_(2);
  reference.state.x_dot = 0.0;
  reference.state.y_dot = 0.0;
  reference.state.z_dot = 0.0;

  reference_pub_.publish(reference);
  std::cout << "Hover point: " << hover_point_.transpose() << std::endl;

  // Give LQR time to get there.
  ros::Duration(hover_duration_).sleep();
  in_flight_ = true;

  // Send the in_flight signal to all other nodes!
  in_flight_pub_.publish(std_msgs::Empty());

  // Return true.
  return true;
}

} //\namespace crazyflie_takeoff
