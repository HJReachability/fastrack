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
// Defines the ValueFunction class. Templated on the vehicle state (VS),
// vehicle control (VC), bound (B), vehicle dynamics (VD) , and planner
// dynamics (PD).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_VALUE_VALUE_FUNCTION_H
#define FASTRACK_VALUE_VALUE_FUNCTION_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>

namespace fastrack {
namespace value {

template< typename VS, typename VC, typename B, typename VD<VS, VC>,
          typename PD<typename PS, typename PC> >
class ValueFunction : private Uncopyable {
public:
  virtual ~ValueFunction() {}

  // Initialize from a ROS NodeHandle.
  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  // Value and gradient at particular states.
  virtual double Value(const VS& x) const = 0;
  virtual VS Gradient(const VS& x) const = 0;

  // Get the optimal control given the vehicle state and planner state.
  inline VC OptimalControl(const VS& vehicle_x, const PS& planner_x) const {
    const VS relative_x = vehicle_x.RelativeTo<PS>(planner_x);
    return vehicle_dynamics_.OptimalControl(relative_x, Gradient(relative_x));
  }

  // Accessors.
  inline const B& TrackingBound() const { return bound_; }
  inline const VD<VS, VC>& VehicleDynamics() const { return vehicle_dynamics_; }
  inline const PD<PS, PC>& VehicleDynamics() const { return planner_dynamics_; }

  // Priority of the optimal control at the given vehicle and planner states.
  // This is a number between 0 and 1, where 1 means the final control signal
  // should be exactly the optimal control signal computed by this
  // value function.
  virtual double Priority(const VS& vehicle_x, const PS& planner_x) const = 0;

protected:
  explicit ValueFunction() {}

  // Load parameters and register callbacks.
  virtual bool LoadParameters(const ros::NodeHandle& n) = 0;
  virtual bool RegisterCallbacks(const ros::NodeHandle& n) = 0;

  // Member variables to be instantiated by derived classes after
  // reading the necessary parameters from the ROS parameter server.
  // Keep a copy of the vehicle and planner dynamics.
  VD<VS, VC> vehicle_dynamics_;
  PD<PS, PC> planner_dynamics_;

  // Keep a copy of the tracking errror bound.
  B bound_;
}; //\class ValueFunction

} //\namespace value
} //\namespace fastrack

#endif
