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

#ifndef FASTRACK_STATE_PLANAR_DUBINS_3D_H
#define FASTRACK_STATE_PLANAR_DUBINS_3D_H

#include <fastrack/state/state.h>

namespace fastrack {
namespace state {

class PlanarDubins3D : public State {
public:
  ~PlanarDubins3D() {}
  explicit PlanarDubins3D()
      : State(), x_(0.0), y_(0.0), theta_(0.0), v_(constants::kDefaultSpeed) {}
  explicit PlanarDubins3D(double x, double y, double theta)
      : State(), x_(x), y_(y), theta_(theta), v_(constants::kDefaultSpeed) {}
  explicit PlanarDubins3D(double x, double y, double theta, double v)
      : State(), x_(x), y_(y), theta_(theta), v_(v) {}
  explicit PlanarDubins3D(const fastrack_msgs::State &msg);
  explicit PlanarDubins3D(const VectorXd &config);

  // Accessors.
  // NOTE! Since this is planar, we assume that Z is a fixed
  //       static variable.
  inline double X() const { return x_; }
  inline double Y() const { return y_; }
  inline double Z() const { return z_; }
  inline double Theta() const { return theta_; }
  inline double V() const { return v_; }
  inline double Vx() const { return v_ * std::cos(theta_); }
  inline double Vy() const { return v_ * std::sin(theta_); }
  inline double Vz() const { return 0.0; }

  inline Vector3d Position() const { return Vector3d(x_, y_, z_); }
  inline Vector3d Velocity() const { return Vector3d(Vx(), Vy(), Vz()); }
  inline VectorXd Configuration() const {
    VectorXd config(2);
    config(0) = x_;
    config(1) = y_;

    return config;
  }

  // Setters.
  inline double &X() { return x_; }
  inline double &Y() { return y_; }
  inline double &Z() { return z_; }
  inline double &Theta() { return theta_; }
  inline double &V() { return v_; }

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
  static void SetBounds(const PlanarDubins3D &lower,
                        const PlanarDubins3D &upper);
  static void SetBounds(const std::vector<double> &lower,
                        const std::vector<double> &upper);
  static const PlanarDubins3D& GetLower();
  static const PlanarDubins3D& GetUpper();
  static VectorXd GetConfigurationLower();
  static VectorXd GetConfigurationUpper();

  // Sample from the configuration space associated with this state space.
  static VectorXd SampleConfiguration();

  // Sample from the state space itself.
  // NOTE! Sets v_ to default value.
  static PlanarDubins3D Sample();

  // Compound assignment operators.
  PlanarDubins3D& operator+=(const PlanarDubins3D& rhs);
  PlanarDubins3D& operator-=(const PlanarDubins3D& rhs);
  PlanarDubins3D& operator*=(double s);
  PlanarDubins3D& operator/=(double s);

  // Binary operators.
  friend PlanarDubins3D operator+(PlanarDubins3D lhs,
                                    const PlanarDubins3D& rhs);
  friend PlanarDubins3D operator-(PlanarDubins3D lhs,
                                    const PlanarDubins3D& rhs);
  friend PlanarDubins3D operator*(PlanarDubins3D lhs, double s);
  friend PlanarDubins3D operator*(double s, PlanarDubins3D rhs);
  friend PlanarDubins3D operator/(PlanarDubins3D lhs, double s);
  friend PlanarDubins3D operator/(double s, PlanarDubins3D rhs);

private:
  // (x, y) position, and heading angle theta.
  double x_;
  double y_;
  double theta_;

  // Parameter for speed.
  // NOTE! This is NOT a state.
  double v_;

  // Static height z.
  static double z_;

  // Static state space bounds for this state space.
  static PlanarDubins3D lower_;
  static PlanarDubins3D upper_;
}; //\class PlanarDubins3D

} // namespace state
} // namespace fastrack

#endif
