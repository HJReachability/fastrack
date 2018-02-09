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
// Class for purely geometric (position + velocity) state.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_STATE_POSITION_VELOCITY_H
#define FASTRACK_STATE_POSITION_VELOCITY_H

#include <fastrack/state/state.h>

namespace fastrack {
namespace state {

class PositionVelocity : public State {
public:
  ~PositionVelocity() {}
  explicit PositionVelocity(double x, double y, double z,
                            double vx, double vy, double vz);
  explicit PositionVelocity(const Vector3d& position,
                            const Vector3d& velocity);

  // Accessors.
  inline double X() const { return position_(0); }
  inline double Y() const { return position_(1); }
  inline double Z() const { return position_(2); }

  inline Vector3d Position() const { return position_; }
  inline Vector3d Velocity() const { return velocity_; }

  // Static function to sample from the configuration space associated
  // with this state space. Pass in the lower and upper bounds from
  // which to sample.
  static VectorXd Sample(const VectorXd& lower, const VectorXd& upper);

  // Compound assignment operators.
  PositionVelocity& operator+=(const PositionVelocity& rhs);
  PositionVelocity& operator-=(const PositionVelocity& rhs);
  PositionVelocity& operator*=(double s);
  PositionVelocity& operator/=(double s);

  // Binary operators.
  friend PositionVelocity operator+(PositionVelocity lhs,
                                    const PositionVelocity& rhs);
  friend PositionVelocity operator-(PositionVelocity lhs,
                                    const PositionVelocity& rhs);
  friend PositionVelocity operator*(PositionVelocity lhs, double s);
  friend PositionVelocity operator*(double s, PositionVelocity rhs);
  friend PositionVelocity operator/(PositionVelocity lhs, double s);
  friend PositionVelocity operator/(double s, PositionVelocity rhs);

private:
  Vector3d position_;
  Vector3d velocity_;
}; //\class PositionVelocity

} //\namespace state
} //\namespace fastrack

#endif
