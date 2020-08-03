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

#include <ros/ros.h>

namespace fastrack {
namespace state {

// Initialize static configuration space bounds.
PositionVelocity PositionVelocity::lower_ =
    PositionVelocity(Vector3d::Zero(), Vector3d::Zero());
PositionVelocity PositionVelocity::upper_ =
    PositionVelocity(Vector3d::Zero(), Vector3d::Zero());

// Constructors.
PositionVelocity::PositionVelocity(double x, double y, double z, double vx,
                                   double vy, double vz)
    : State(), position_(Vector3d(x, y, z)), velocity_(Vector3d(vx, vy, vz)) {}
PositionVelocity::PositionVelocity(const Vector3d &position,
                                   const Vector3d &velocity)
    : State(), position_(position), velocity_(velocity) {}
PositionVelocity::PositionVelocity(const fastrack_msgs::State &msg)
    : State(), position_(Vector3d::Zero()), velocity_(Vector3d::Zero()) {
  // Check dimensions.
  if (msg.x.size() == StateDimension()) {
    position_(0) = msg.x[0];
    position_(1) = msg.x[1];
    position_(2) = msg.x[2];
    velocity_(0) = msg.x[3];
    velocity_(1) = msg.x[4];
    velocity_(2) = msg.x[5];
  } else if (msg.x.size() == ConfigurationDimension()) {
    position_(0) = msg.x[0];
    position_(1) = msg.x[1];
    position_(2) = msg.x[2];
  } else {
    ROS_ERROR_THROTTLE(1.0,
                       "PositionVelocity: msg dimension is incorrect: %zu.",
                       msg.x.size());
  }
}
PositionVelocity::PositionVelocity(const VectorXd &x)
    : State(), position_(Vector3d::Zero()), velocity_(Vector3d::Zero()) {
  // Check dimensions.
  if (x.size() == ConfigurationDimension()) {
    position_(0) = x(0);
    position_(1) = x(1);
    position_(2) = x(2);
  } else if (x.size() == StateDimension()) {
    position_(0) = x(0);
    position_(1) = x(1);
    position_(2) = x(2);
    velocity_(0) = x(3);
    velocity_(1) = x(4);
    velocity_(2) = x(5);
  } else {
    throw std::runtime_error("PositionVelocity: incorrect input dimension.");
  }
}

// Set non-configuration dimensions to match the given config derivative.
void PositionVelocity::SetConfigurationDot(const VectorXd &configuration_dot) {
  if (configuration_dot.size() != ConfigurationDimension())
    throw std::runtime_error(
        "PositionVelocity: invalid configuration_dot dim.");

  velocity_(0) = configuration_dot(0);
  velocity_(1) = configuration_dot(1);
  velocity_(2) = configuration_dot(2);
}

// Static function to sample from the configuration space associated
// with this state space. Pass in the lower and upper bounds from
// which to sample.
VectorXd PositionVelocity::SampleConfiguration() {
  const size_t kConfigurationSpaceDimension = 3;

  // Initialize a uniform random distribution in (0, 1).
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Extract lower/upper configuration bounds.
  const VectorXd lower_config = PositionVelocity::GetConfigurationLower();
  const VectorXd upper_config = PositionVelocity::GetConfigurationUpper();

  // Generate random sample.
  VectorXd sample(kConfigurationSpaceDimension);
  for (size_t ii = 0; ii < kConfigurationSpaceDimension; ii++)
    sample(ii) =
        lower_config(ii) + (upper_config(ii) - lower_config(ii)) * unif(rng_);

  return sample;
}

// Sample from the state space itself.
PositionVelocity PositionVelocity::Sample() {
  // Initialize a uniform random distribution in (0, 1).
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Generate random sample.
  const double x = lower_.position_(0) +
                   (upper_.position_(0) - lower_.position_(0)) * unif(rng_);
  const double y = lower_.position_(1) +
                   (upper_.position_(1) - lower_.position_(1)) * unif(rng_);
  const double z = lower_.position_(2) +
                   (upper_.position_(2) - lower_.position_(2)) * unif(rng_);
  const double vx = lower_.velocity_(0) +
                    (upper_.velocity_(0) - lower_.velocity_(0)) * unif(rng_);
  const double vy = lower_.velocity_(1) +
                    (upper_.velocity_(1) - lower_.velocity_(1)) * unif(rng_);
  const double vz = lower_.velocity_(2) +
                    (upper_.velocity_(2) - lower_.velocity_(2)) * unif(rng_);

  return PositionVelocity(x, y, z, vx, vy, vz);
}

// Samples within a box of the given position, intersected with the state space
// bounds.
PositionVelocity PositionVelocity::SampleCloseTo(const Vector3d &pos,
                                                 double d) {
  std::uniform_real_distribution<double> x_dist(
      std::max(lower_.position_.x(), pos.x() - d),
      std::min(upper_.position_.x(), pos.x() + d));
  std::uniform_real_distribution<double> y_dist(
      std::max(lower_.position_.y(), pos.y() - d),
      std::min(upper_.position_.y(), pos.y() + d));
  std::uniform_real_distribution<double> z_dist(
      std::max(lower_.position_.z(), pos.z() - d),
      std::min(upper_.position_.z(), pos.z() + d));

  // HACK! Set velocity to zero since that's what we want most of the time.
  return PositionVelocity(x_dist(rng_), y_dist(rng_), z_dist(rng_), 0.0, 0.0,
                          0.0);
}

// For a given configuration, what are the corresponding positions in
// position space that the system occupies.
// NOTE! For simplicity, this is a finite set. In future, this could
// be generalized to a collection of generic obstacles.
std::vector<Vector3d> PositionVelocity::OccupiedPositions() const {
  return std::vector<Vector3d>({position_});
}

// Set bounds of the state space.
void PositionVelocity::SetBounds(const PositionVelocity &lower,
                                 const PositionVelocity &upper) {
  lower_ = lower;
  upper_ = upper;
}

// Set state space bounds from std vectors. Layout is assumed to be
// [x, y, z, vx, vy, vz].
void PositionVelocity::SetBounds(const std::vector<double> &lower,
                                 const std::vector<double> &upper) {
  // Check dimensions.
  if (lower.size() != 6 || upper.size() != 6)
    throw std::runtime_error("PositionVelocity: bad bound dimension.");

  lower_.position_(0) = lower[0];
  lower_.position_(1) = lower[1];
  lower_.position_(2) = lower[2];
  lower_.velocity_(0) = lower[3];
  lower_.velocity_(1) = lower[4];
  lower_.velocity_(2) = lower[5];

  upper_.position_(0) = upper[0];
  upper_.position_(1) = upper[1];
  upper_.position_(2) = upper[2];
  upper_.velocity_(0) = upper[3];
  upper_.velocity_(1) = upper[4];
  upper_.velocity_(2) = upper[5];
}

// Convert from VectorXd. Assume State is [x, y, z, vx, vy, vz].
void PositionVelocity::FromVector(const VectorXd &x) {
  if (x.size() == 6) {
    position_(0) = x(0);
    position_(1) = x(1);
    position_(2) = x(2);
    velocity_(0) = x(3);
    velocity_(1) = x(4);
    velocity_(2) = x(5);
    return;
  }

  ROS_ERROR("PositionVelocity: vector is of the wrong size.");
}

// Convert to VectorXd. Assume State is [x, y, z, vx, vy, vz].
VectorXd PositionVelocity::ToVector() const {
  VectorXd x(6);
  x(0) = position_(0);
  x(1) = position_(1);
  x(2) = position_(2);
  x(3) = velocity_(0);
  x(4) = velocity_(1);
  x(5) = velocity_(2);

  return x;
}

// Convert from ROS message. Assume State is [x, y, z, vx, vy, vz] or
// configuration only.
void PositionVelocity::FromRos(const fastrack_msgs::State &msg) {
  if (msg.x.size() == 6) {
    // Message contains full state.
    position_(0) = msg.x[0];
    position_(1) = msg.x[1];
    position_(2) = msg.x[2];
    velocity_(0) = msg.x[3];
    velocity_(1) = msg.x[4];
    velocity_(2) = msg.x[5];
  } else if (msg.x.size() == 3) {
    // Message contains configuration only.
    position_(0) = msg.x[0];
    position_(1) = msg.x[1];
    position_(2) = msg.x[2];
  } else
    ROS_ERROR("PositionVelocity: msg is neither state nor configuration.");
}

// Convert to ROS message.
fastrack_msgs::State PositionVelocity::ToRos() const {
  fastrack_msgs::State msg;
  msg.x.push_back(position_(0));
  msg.x.push_back(position_(1));
  msg.x.push_back(position_(2));
  msg.x.push_back(velocity_(0));
  msg.x.push_back(velocity_(1));
  msg.x.push_back(velocity_(2));

  return msg;
}

// Get bounds of state space.
const PositionVelocity &PositionVelocity::GetLower() { return lower_; }
const PositionVelocity &PositionVelocity::GetUpper() { return upper_; }

// Get bounds of configuration space.
VectorXd PositionVelocity::GetConfigurationLower() {
  return lower_.Configuration();
}
VectorXd PositionVelocity::GetConfigurationUpper() {
  return upper_.Configuration();
}

// Compound assignment operators.
PositionVelocity &PositionVelocity::operator+=(const PositionVelocity &rhs) {
  position_ += rhs.position_;
  velocity_ += rhs.velocity_;
  return *this;
}

PositionVelocity &PositionVelocity::operator-=(const PositionVelocity &rhs) {
  position_ -= rhs.position_;
  velocity_ -= rhs.velocity_;
  return *this;
}

PositionVelocity &PositionVelocity::operator*=(double s) {
  position_ *= s;
  velocity_ *= s;
  return *this;
}

PositionVelocity &PositionVelocity::operator/=(double s) {
  position_ /= s;
  velocity_ /= s;
  return *this;
}

// Binary operators.
PositionVelocity operator+(PositionVelocity lhs, const PositionVelocity &rhs) {
  lhs += rhs;
  return lhs;
}

PositionVelocity operator-(PositionVelocity lhs, const PositionVelocity &rhs) {
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

}  // namespace state
}  // namespace fastrack
