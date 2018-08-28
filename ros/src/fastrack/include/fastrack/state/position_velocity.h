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

#include <fastrack/state/planar_dubins_3d.h>
#include <fastrack/state/state.h>

namespace fastrack {
namespace state {

class PositionVelocity : public State {
public:
  ~PositionVelocity() {}
  explicit PositionVelocity()
      : position_(Vector3d::Zero()), velocity_(Vector3d::Zero()) {}
  explicit PositionVelocity(double x, double y, double z, double vx, double vy,
                            double vz);
  explicit PositionVelocity(const Vector3d &position, const Vector3d &velocity);
  explicit PositionVelocity(const fastrack_msgs::State &msg);
  explicit PositionVelocity(const VectorXd &x);

  // Accessors.
  inline double X() const { return position_(0); }
  inline double Y() const { return position_(1); }
  inline double Z() const { return position_(2); }
  inline double Vx() const { return velocity_(0); }
  inline double Vy() const { return velocity_(1); }
  inline double Vz() const { return velocity_(2); }

  inline Vector3d Position() const { return position_; }
  inline Vector3d Velocity() const { return velocity_; }
  inline VectorXd Configuration() const {
    VectorXd config(3);
    config(0) = position_(0);
    config(1) = position_(1);
    config(2) = position_(2);

    return config;
  }

  // Setters.
  inline double &X() { return position_(0); }
  inline double &Y() { return position_(1); }
  inline double &Z() { return position_(2); }
  inline double &Vx() { return velocity_(0); }
  inline double &Vy() { return velocity_(1); }
  inline double &Vz() { return velocity_(2); }

  inline Vector3d &Position() { return position_; }
  inline Vector3d &Velocity() { return velocity_; }

  // Set non-configuration dimensions to match the given config derivative.
  void SetConfigurationDot(const VectorXd &configuration_dot);

  // What are the positions that the system occupies at the current state.
  // NOTE! For simplicity, this is a finite set. In future, this could
  // be generalized to a collection of generic obstacles.
  std::vector<Vector3d> OccupiedPositions() const;

  // Convert from/to VectorXd. Assume State is [x, y, z, vx, vy, vz]
  void FromVector(const VectorXd &x);
  VectorXd ToVector() const;

  // Convert from/to ROS message. Assume State is [x, y, z, vx, vy, vz]
  // or configuration only.
  void FromRos(const fastrack_msgs::State::ConstPtr &msg);
  fastrack_msgs::State ToRos() const;

  // Dimension of the state and configuration spaces.
  static constexpr size_t StateDimension() { return 6; }
  static constexpr size_t ConfigurationDimension() { return 3; }

  // Set/get bounds of the state/configuration space.
  static void SetBounds(const PositionVelocity &lower,
                        const PositionVelocity &upper);
  static void SetBounds(const std::vector<double> &lower,
                        const std::vector<double> &upper);
  static const PositionVelocity& GetLower();
  static const PositionVelocity& GetUpper();
  static VectorXd GetConfigurationLower();
  static VectorXd GetConfigurationUpper();

  // Sample from the configuration space associated with this state space.
  static VectorXd SampleConfiguration();

  // Sample from the state space itself.
  static PositionVelocity Sample();

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

  // Static state space bounds for this state space.
  static PositionVelocity lower_;
  static PositionVelocity upper_;
}; //\class PositionVelocity

} // namespace state
} // namespace fastrack

#endif
