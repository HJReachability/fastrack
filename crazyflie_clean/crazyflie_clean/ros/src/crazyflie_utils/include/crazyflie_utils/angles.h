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
// Angle manipulation utilities.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_UTILS_ANGLES_H
#define CRAZYFLIE_UTILS_ANGLES_H

#include <crazyflie_utils/types.h>

namespace crazyflie_utils {
namespace angles {
  // Convert degrees to radians.
  static inline double DegreesToRadians(double d) {
    return d * M_PI / 180.0;
  }

  // Convert radians to degrees.
  static inline double RadiansToDegrees(double r) {
    return r * 180.0 / M_PI;
  }

  // Wrap angle in degrees to [-180, 180].
  static inline double WrapAngleDegrees(double d) {
    d = std::fmod(d + 180.0, 360.0) - 180.0;

    if (d < -180.0)
      d += 360.0;

    return d;
  }

  // Wrap angle in radians to [-pi, pi].
  static inline double WrapAngleRadians(double r) {
    r = std::fmod(r + M_PI, 2.0 * M_PI) - M_PI;

    if (r < -M_PI)
      r += 2.0 * M_PI;

    return r;
  }

  // Convert rotation matrix to roll-pitch-yaw Euler angles with
  // aerospace convention:
  static inline Eigen::Vector3d Matrix2RPY(const Eigen::Matrix3d& R) {
    const double roll = std::atan2(-R(1,2), R(2,2));
    const double pitch = std::asin (R(0,2));
    const double yaw = std::atan2(-R(0,1), R(0,0));

    return Vector3d(roll, pitch, yaw);
  }

} //\namespace angles
} //\namespace crazyflie_utils

#endif
