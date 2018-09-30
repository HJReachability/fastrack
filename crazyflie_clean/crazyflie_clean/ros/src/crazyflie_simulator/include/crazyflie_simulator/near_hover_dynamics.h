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
// Near hover forward dynamics.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_SIMULATOR_NEAR_HOVER_DYNAMICS_H
#define CRAZYFLIE_SIMULATOR_NEAR_HOVER_DYNAMICS_H

#include <crazyflie_simulator/forward_dynamics.h>
#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/ControlStamped.h>

#include <ros/ros.h>
#include <math.h>
#include <fstream>

namespace crazyflie_simulator {

class NearHoverDynamics : public ForwardDynamics {
public:
  virtual ~NearHoverDynamics() {}
  explicit NearHoverDynamics()
    : ForwardDynamics() {}

  // Evaluate forward dynamics at a particular state.
  inline VectorXd operator()(const VectorXd& x, const VectorXd& u) const {
    VectorXd x_dot(7);

    // Approximate dynamics.
    x_dot(0) = x(3);
    x_dot(1) = x(4);
    x_dot(2) = x(5);
    x_dot(3) = crazyflie_utils::constants::G * std::tan(u(1));
    x_dot(4) = -crazyflie_utils::constants::G * std::tan(u(0));
    x_dot(5) = u(3) - crazyflie_utils::constants::G;
    x_dot(6) = u(2);

#if 0
    // Actual dynamics.
    x_dot(0) = x(3);
    x_dot(1) = x(4);
    x_dot(2) = x(5);
    x_dot(3) = u(3)*std::sin(u(1))*std::cos(x(6)) - u(3)*std::sin(u(0))*std::sin(x(6));
    x_dot(4) = u(3)*std::sin(u(0))*std::cos(x(6)) + u(3)*std::sin(u(1))*std::sin(x(6));
    x_dot(5) = u(3)*std::cos(u(0))*std::cos(u(1)) - crazyflie_utils::constants::G;
    x_dot(6) = u(2);
#endif

    return x_dot;
  }
}; //\class NearHoverDynamics

} //\namespace crazyflie_simulator

#endif
