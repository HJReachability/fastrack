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
// Base class for all kinematic planners. Geometric planners are planners that
// operate directly in the configuration space and use only kinematics instead
// of dynamics.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_KINEMATIC_PLANNER_H
#define FASTRACK_PLANNING_KINEMATIC_PLANNER_H

#include <fastrack/planning/planner.h>
#include <fastrack/dynamics/kinematics.h>

namespace fastrack {
namespace planning {

using dynamics::Kinematics;

template<typename S, typename B>
class KinematicPlanner : public Planner< S, Kinematics<S>, B > {
public:
  virtual ~KinematicPlanner() {}

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  virtual Trajectory<S> Plan(
    const S& start, const S& goal, const Environment& env,
    double start_time=0.0) const = 0;

protected:
  explicit KinematicPlanner(const Kinematics<S>& dynamics, const B& bound)
    : Planner(dynamics, bound) {}
}; //\class KinematicPlanner

} //\namespace planning
} //\namespace fastrack

#endif
