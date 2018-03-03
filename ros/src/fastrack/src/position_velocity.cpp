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

#include <fastrack/state/position_velocity.h>



namespace fastrack {
namespace state {

// Initialize static configuration space bounds.
VectorXd PositionVelocity::lower_ = VectorXd::Zero(ConfigurationDimension());
VectorXd PositionVelocity::upper_ = VectorXd::Ones(ConfigurationDimension());

// Constructors.
PositionVelocity::PositionVelocity(double x, double y, double z,
                                   double vx, double vy, double vz)
  : State(),
    position_(Vector3d(x, y, z)),
    velocity_(Vector3d(vx, vy, vz)) {}
PositionVelocity::PositionVelocity(const Vector3d& position,
                                   const Vector3d& velocity)
  : State(),
    position_(position),
    velocity_(velocity) {}
PositionVelocity::PositionVelocity(const fastrack_msgs::State& msg)
  : State(),
    position_(Vector3d::Zero()),
    velocity_(Vector3d::Zero()) {
  // Check dimensions.
  if (msg.x.size() != StateDimension())
    throw std::runtime_error("PositionVelocity: incorrect msg dimension.");

  position_(0) = msg.x[0];
  position_(1) = msg.x[1];
  position_(2) = msg.x[2];
  velocity_(0) = msg.x[3];
  velocity_(1) = msg.x[4];
  velocity_(2) = msg.x[5];
}
PositionVelocity::PositionVelocity(const VectorXd& config)
  : State(),
    position_(Vector3d::Zero()),
    velocity_(Vector3d::Zero()) {
  // Check dimensions.
  if (config.size() != ConfigurationDimension())
    throw std::runtime_error("PositionVelocity: incorrect config dimension.");

  position_(0) = config(0);
  position_(1) = config(1);
  position_(2) = config(2);
}

// Static function to sample from the configuration space associated
// with this state space. Pass in the lower and upper bounds from
// which to sample.
VectorXd PositionVelocity::Sample() {
  const size_t kConfigurationSpaceDimension = 3;

  // Make sure bounds are of the right size.
  if (lower_.size() != kConfigurationSpaceDimension ||
      upper_.size() != kConfigurationSpaceDimension) {
    throw std::runtime_error("PositionVelocity: invalid configuration space bounds.");
  }

  // Initialize a uniform random distribution in (0, 1).
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Generate random sample.
  VectorXd sample(kConfigurationSpaceDimension);
  for (size_t ii = 0; ii < kConfigurationSpaceDimension; ii++)
    sample(ii) = lower_(ii) + (upper_(ii) - lower_(ii)) * unif(rng_);

  return sample;
}

// For a given configuration, what are the corresponding positions in
// position space that the system occupies.
// NOTE! For simplicity, this is a finite set. In future, this could
// be generalized to a collection of generic obstacles.
std::vector<Vector3d> PositionVelocity::OccupiedPositions() const {
  return std::vector<Vector3d>({ position_ });
}

// Set bounds of the configuration space.
void PositionVelocity::SetConfigurationBounds(
  const VectorXd& lower, const VectorXd& upper) {
  // Catch incorrect dimensions.
  if (lower.size() != ConfigurationDimension() ||
      upper.size() != ConfigurationDimension())
    throw std::runtime_error("PositionVelocity: incorrect bound dimensions.");

  lower_ = lower;
  upper_ = upper;
}

// Convert to ROS message.
fastrack_msgs::State PositionVelocity::ToRos() const {
  fastrack_msgs::State msg;
  msg.x.push_back(position_[0]);
  msg.x.push_back(position_[1]);
  msg.x.push_back(position_[2]);
  msg.x.push_back(velocity_[0]);
  msg.x.push_back(velocity_[1]);
  msg.x.push_back(velocity_[2]);

  return msg;
}


// Get bounds of configuration space.
VectorXd PositionVelocity::GetConfigurationLower() { return lower_; }
VectorXd PositionVelocity::GetConfigurationUpper() { return upper_; }

// Compound assignment operators.
PositionVelocity& PositionVelocity::operator+=(const PositionVelocity& rhs) {
  position_ += rhs.position_;
  velocity_ += rhs.velocity_;
  return *this;
}

PositionVelocity& PositionVelocity::operator-=(const PositionVelocity& rhs) {
  position_ -= rhs.position_;
  velocity_ -= rhs.velocity_;
  return *this;
}

PositionVelocity& PositionVelocity::operator*=(double s) {
  position_ *= s;
  velocity_ *= s;
  return *this;
}

PositionVelocity& PositionVelocity::operator/=(double s) {
  position_ /= s;
  velocity_ /= s;
  return *this;
}

// Binary operators.
PositionVelocity operator+(PositionVelocity lhs, const PositionVelocity& rhs) {
  lhs += rhs;
  return lhs;
}

PositionVelocity operator-(PositionVelocity lhs, const PositionVelocity& rhs) {
  lhs -= rhs;
  return lhs;
}

PositionVelocity operator*(PositionVelocity lhs, double s) {
  lhs *= s;
  return lhs;
}

PositionVelocity operator*(double s, PositionVelocity rhs) {
  rhs *= s;
  return rhs;
}

PositionVelocity operator/(PositionVelocity lhs, double s) {
  lhs /= s;
  return lhs;
}

PositionVelocity operator/(double s, PositionVelocity rhs) {
  rhs /= s;
  return rhs;
}

} //\namespace state
} //\namespace fastrack

