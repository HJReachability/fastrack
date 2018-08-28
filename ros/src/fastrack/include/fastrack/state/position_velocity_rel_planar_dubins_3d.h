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
 *          Jaime F. Fisac         ( jfisac@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Class for relative state computed between PositionVelocity and
// PlanarDubins3D.
//
// NOTE! Because this is a relative state, we do NOT support
// sampling or configuration space conversions.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_STATE_POSITION_VELOCITY_REL_PLANAR_DUBINS_3D_H
#define FASTRACK_STATE_POSITION_VELOCITY_REL_PLANAR_DUBINS_3D_H

#include <fastrack/state/relative_state.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/state/planar_dubins_3d.h>

namespace fastrack {
namespace state {

class PositionVelocityRelPlanarDubins3D
    : public RelativeState<PositionVelocity, PlanarDubins3D> {
public:
  ~PositionVelocityRelPlanarDubins3D() {}

  // NOTE! We do NOT provide a default constructor. Objects from this
  // class SHOULD be initialized from the two corresponding states.
  explicit PositionVelocityRelPlanarDubins3D(const PositionVelocity &tracker_x,
                                             const PlanarDubins3D &planner_x)
      : distance_(std::hypot(tracker_x.X() - planner_x.X(),
                             tracker_x.Y() - planner_x.Y())),
        bearing_(std::atan2(tracker_x.Y() - planner_x.Y(),
                            tracker_x.X() - planner_x.X())),
        tangent_velocity_(std::cos(planner_x.Theta()) * tracker_x.Vx() +
                          std::sin(planner_x.Theta()) * tracker_x.Vy()),
        normal_velocity_(-std::sin(planner_x.Theta()) * tracker_x.Vx() +
                          std::cos(planner_x.Theta()) * tracker_x.Vy()) {}

  // Construct from VectorXd.
  explicit PositionVelocityRelPlanarDubins3D(const VectorXd &x)
    : RelativeState<PositionVelocity, PlanarDubins3D>() {
    FromVector(x);
  }

  // Construct directly.
  explicit PositionVelocityRelPlanarDubins3D(double distance, double bearing,
                                             double tangent_v, double normal_v)
      : distance_(distance), bearing_(bearing), tangent_velocity_(tangent_v),
        normal_velocity_(normal_v) {}

  // Convert from/to VectorXd.
  // Assume vector is laid out as follows:
  // [distance, bearing, tangent velocity, normal velocity]
  void FromVector(const VectorXd& x) {
    if (x.size() != StateDimension())
      throw std::runtime_error("Vector was of incorrect dimension.");

    distance_ = x(0);
    bearing_ = x(1);
    tangent_velocity_ = x(2);
    normal_velocity_ = x(3);
  }

  VectorXd ToVector() const {
    VectorXd vec(4);
    vec(0) = distance_;
    vec(1) = bearing_;
    vec(2) = tangent_velocity_;
    vec(3) = normal_velocity_;

    return vec;
  }

  // Dimension of the state space.
  static constexpr size_t StateDimension() { return 4; }

  // Accessors.
  inline double Distance() const { return distance_; }
  inline double Bearing() const { return bearing_; }
  inline double TangentVelocity() const { return tangent_velocity_; }
  inline double NormalVelocity() const { return normal_velocity_; }

private:
  // Relative (x, y) distance.
  double distance_;

  // Relative bearing in the (x, y) plane.
  double bearing_;

  // PositionVelocity's absolute velocity along PlanarDubins3D's heading.
  // (in Frenet frame).
  double tangent_velocity_;

  // PositionVelocity's absolute velocity normal to PlanarDubins3D's heading.
  // (in Frenet frame).
  double normal_velocity_;
}; //\class PositionVelocityRelPlanarDubins3D

} // namespace state
} // namespace fastrack

#endif
