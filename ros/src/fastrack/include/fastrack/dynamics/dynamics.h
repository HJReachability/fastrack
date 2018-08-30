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
// Defines the Dynamics class. Templated on the state type and control type,
// control bound type (CB), as well as the type of service response (SR) which
// encodes these dynamics.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_DYNAMICS_H
#define FASTRACK_DYNAMICS_DYNAMICS_H

#include <fastrack/control/control_bound.h>
#include <fastrack/utils/types.h>

#include <memory>

namespace fastrack {
namespace dynamics {

template <typename S, typename C, typename CB, typename SR>
class Dynamics {
 public:
  // Destructor.
  virtual ~Dynamics() {}

  // Initialize.
  void Initialize(const CB &bound) { control_bound_.reset(new CB(bound)); }
  void Initialize(const std::vector<double> &params) {
    control_bound_.reset(new CB(params));
  }

  // Derived classes must be able to give the time derivative of state
  // as a function of current state and control.
  virtual S Evaluate(const S &x, const C &u) const = 0;

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the specified state.
  virtual C OptimalControl(const S &x, const S &value_gradient) const = 0;

  // Accessor for control bound.
  const CB &GetControlBound() const {
    if (!control_bound_)
      throw std::runtime_error("Dynamics was not initialized.");

    return *control_bound_;
  }

  // Convert to the appropriate service response type.
  virtual SR ToRos() const = 0;

  // Convert from the appropriate service response type.
  virtual void FromRos(const SR &res) = 0;

 protected:
  explicit Dynamics() {}
  explicit Dynamics(const CB &bound) : control_bound_(new CB(bound)) {}
  explicit Dynamics(const std::vector<double> &params)
      : control_bound_(new CB(params)) {}

  // Control bound.
  std::unique_ptr<CB> control_bound_;
};  //\class Dynamics

}  // namespace dynamics
}  // namespace fastrack

#endif
