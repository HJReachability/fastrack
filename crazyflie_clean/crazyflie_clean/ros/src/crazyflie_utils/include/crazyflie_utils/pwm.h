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
// Convert thrust to PWM signal.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_UTILS_PWM_H
#define CRAZYFLIE_UTILS_PWM_H

#include <crazyflie_utils/types.h>

#include <limits>
#include <ros/ros.h>

namespace crazyflie_utils {
namespace pwm {
  // Convert the given thrust to a PWM signal (still double).
  static inline double ThrustToPwmDouble(double thrust) {
    const double k_thrust = 42000.0 / constants::G; //40000.0 / constants::G;
    return thrust * k_thrust;
  }

  // Get the PWM signal (uint16) to generate the given thrust.
  static inline uint16_t ThrustToPwmUnsignedShort(double thrust) {
    const double control = ThrustToPwmDouble(thrust);

#ifdef ENABLE_DEBUG_MESSAGES
    if (control > static_cast<double>(std::numeric_limits<uint16_t>::max())) {
      ROS_WARN("Desired thrust is too high. Sending max PWM signal instead.");
      return  std::numeric_limits<uint16_t>::max();
    }
#endif

    return static_cast<uint16_t>(control);
  }

} //\namespace pwm
} //\namespace crazyflie_utils

#endif
