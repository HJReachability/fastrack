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
// Defines the RelativeDynamics class, which encodes relative dynamcis between
// the tracker and the planner. Templated on both state types, and also the
// control types for each system.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_RELATIVE_DYNAMICS_H
#define FASTRACK_DYNAMICS_RELATIVE_DYNAMICS_H

#include <fastrack/control/control_bound.h>
#include <fastrack/state/relative_state.h>
#include <fastrack/utils/types.h>

namespace fastrack {
namespace dynamics {

template <typename TS, typename TC, typename PS, typename PC>
class RelativeDynamics {
public:
  // Destructor.
  virtual ~RelativeDynamics() {}

  // Derived classes must be able to give the time derivative of relative state
  // as a function of current state and control of each system.
  virtual RelativeState<TS, PS> Evaluate(const TS &tracker_x,
                                         const TC &tracker_u,
                                         const PS &planner_x,
                                         const PC &planner_u) const = 0;

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the relative state specified
  // by the given system states, provided abstract control bounds.
  virtual TC OptimalControl(const TS &tracker_x, const PS &planner_x,
                            const RelativeState<TS, PS> &value_gradient,
                            const ControlBound<TC> &tracker_u_bound,
                            const ControlBound<PC> &planner_u_bound) const = 0;

protected:
  explicit RelativeDynamics() {}
}; //\class RelativeDynamics

} // namespace dynamics
} // namespace fastrack

#endif
