/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
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
// 4D quadrotor controls as a demo control type. All controls must support
// functions like min and max of different control variables.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CONTROL_CONTROL_H
#define FASTRACK_CONTROL_CONTROL_H

#include <fastrack/utils/types.h>
#include <fastrack_msgs/Control.h>

namespace fastrack {
namespace control {

struct QuadrotorControl {
  // Pitch, roll, yaw rate, thrust.
  double pitch;
  double roll;
  double yaw_rate;
  double thrust;

  // Constructors and destructor.
  ~QuadrotorControl() {}
  explicit QuadrotorControl() {}
  explicit QuadrotorControl(double p, double r, double yr, double t)
    : pitch(p),
      roll(r),
      yaw_rate(yr),
      thrust(t) {}

  // Convert to ROS message. Assume ordering [pitch, roll, yaw_rate, thrust].
  // NOTE! Set priority to 1 by default.
  inline fastrack_msgs::Control ToRos(double priority=1.0) {
    fastrack_msgs::Control msg;
    msg.u.push_back(pitch);
    msg.u.push_back(roll);
    msg.u.push_back(yaw_rate);
    msg.u.push_back(thrust);
    msg.priority = priority;

    // std::cout << "optimal control (pitch, roll, yr, thrust)" << pitch << ", " << roll << ", " << yaw_
      // rate << ", " << thrust << std::endl;

    return msg;
  }
}; //\struct QuadrotorControl

} //\namespace control
} //\namespace fastrack

#endif
