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
// Class for relative state computed between two PositionVelocity states.
//
// NOTE! Because this is a relative state, we do NOT support
// sampling or configuration space conversions.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_STATE_POSITION_VELOCITY_REL_POSITION_VELOCITY_H
#define FASTRACK_STATE_POSITION_VELOCITY_REL_POSITION_VELOCITY_H

#include <fastrack/state/position_velocity.h>
#include <fastrack/state/relative_state.h>

namespace fastrack {
namespace state {

class PositionVelocityRelPositionVelocity
    : public RelativeState<PositionVelocity, PositionVelocity> {
public:
  ~PositionVelocityRelPositionVelocity() {}

  // NOTE! We do NOT provide a default constructor. Objects from this
  // class SHOULD be initialized from the two corresponding states.
  explicit PositionVelocityRelPositionVelocity(
      const PositionVelocity &tracker_x, const PositionVelocity &planner_x)
      : RelativeState<PositionVelocity, PositionVelocity>(),
        x_(tracker_x.Position() - planner_x.Position(),
           tracker_x.Velocity() - planner_x.Velocity()) {}

  // Construct from a PositionVelocity, or from separate position
  // and velocity vectors.
  explicit PositionVelocityRelPositionVelocity(const PositionVelocity &other)
      : RelativeState<PositionVelocity, PositionVelocity>(), x_(other) {}
  explicit PositionVelocityRelPositionVelocity(const Vector3d &position,
                                               const Vector3d &velocity)
      : RelativeState<PositionVelocity, PositionVelocity>(),
        x_(position, velocity) {}

  // Dimension of the state space.
  static constexpr size_t StateDimension() { return 6; }

  // Accessors.
  inline const PositionVelocity &State() const { return x_; }

private:
  // Relative state is just a single PositionVelocity.
  const PositionVelocity x_;
}; //\class PositionVelocityRelPositionVelocity

} // namespace state
} // namespace fastrack

#endif
