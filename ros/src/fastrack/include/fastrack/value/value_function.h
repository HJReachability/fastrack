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
// Defines the ValueFunction class. Templated on the tracker/planner state
// (TS/PS), tracker/planner control (TC/PC), tracker/planner dynamics (TD/PC),
// and bound (B).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_VALUE_VALUE_FUNCTION_H
#define FASTRACK_VALUE_VALUE_FUNCTION_H

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/relative_state.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>

namespace fastrack {
namespace value {

using dynamics::Dynamics;
using dynamics::RelativeDynamics;
using state::RelativeState;

template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename B>
class ValueFunction : private Uncopyable {
 public:
  virtual ~ValueFunction() {}

  // Initialize from a ROS NodeHandle.
  inline bool Initialize(const ros::NodeHandle& n) {
    name_ = ros::names::append(n.getNamespace(), "ValueFunction");

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

  // Value and gradient at particular relative states.
  virtual double Value(const TS& tracker_x, const PS& planner_x) const = 0;
  virtual std::unique_ptr<RelativeState<TS, PS>> Gradient(
      const TS& tracker_x, const PS& planner_x) const = 0;

  // Get the optimal control given the tracker state and planner state.
  inline TC OptimalControl(const TS& tracker_x, const PS& planner_x) const {
    if (!initialized_)
      throw std::runtime_error("Uninitialized call to OptimalControl.");

    //    std::cout << "Value is: " << Value(tracker_x, planner_x) << std::endl;

    return relative_dynamics_->OptimalControl(
        tracker_x, planner_x, *Gradient(tracker_x, planner_x),
        tracker_dynamics_.GetControlBound(),
        planner_dynamics_.GetControlBound());
  }

  // Accessors.
  inline const B& TrackingBound() const { return bound_; }
  inline const TD& TrackerDynamics() const { return tracker_dynamics_; }
  inline const PD& PlannerDynamics() const { return planner_dynamics_; }
  inline const RelativeDynamics<TS, TC, PS, PC>& GetRelativeDynamics() const {
    if (!initialized_)
      throw std::runtime_error("Uninitialized call to GetRelativeDynamics.");

    return *relative_dynamics_;
  }

  // Priority of the optimal control at the given tracker and planner states.
  // This is a number between 0 and 1, where 1 means the final control signal
  // should be exactly the optimal control signal computed by this
  // value function.
  virtual double Priority(const TS& tracker_x, const PS& planner_x) const = 0;

 protected:
  explicit ValueFunction() : initialized_(false) {}

  // Load parameters and register callbacks. May be overridden.
  virtual bool LoadParameters(const ros::NodeHandle& n) { return true; }
  virtual bool RegisterCallbacks(const ros::NodeHandle& n) { return true; }

  // Member variables to be instantiated by derived classes after
  // reading the necessary parameters from the ROS parameter server.
  // Keep a copy of the tracker and planner dynamics, and a pointer
  // to the relative dynamics.
  TD tracker_dynamics_;
  PD planner_dynamics_;
  std::unique_ptr<RelativeDynamics<TS, TC, PS, PC>> relative_dynamics_;

  // Keep a copy of the tracking errror bound.
  B bound_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};  //\class ValueFunction

}  // namespace value
}  // namespace fastrack

#endif
