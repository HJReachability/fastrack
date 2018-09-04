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
// Class for planar Dubins dynamics with fixed speed.
// TODO: check state bounds when creating or modifying states.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/state/planar_dubins_3d.h>

#include <ros/ros.h>

namespace fastrack {
namespace state {

// Initialize static state space bounds.
PlanarDubins3D PlanarDubins3D::lower_ = PlanarDubins3D();
PlanarDubins3D PlanarDubins3D::upper_ = PlanarDubins3D();

// Initialize static height z_.
double PlanarDubins3D::z_ = constants::kDefaultHeight;

// Constructors.
PlanarDubins3D::PlanarDubins3D(const fastrack_msgs::State &msg)
    : State(), x_(0.0), y_(0.0), theta_(0.0), v_(constants::kDefaultHeight) {
  // Check dimensions.
  if (msg.x.size() == StateDimension()) {
    x_ = msg.x[0];
    y_ = msg.x[1];
    theta_ = msg.x[2];
  } else if (msg.x.size() == StateDimension() + 1) {
    x_ = msg.x[0];
    y_ = msg.x[1];
    theta_ = msg.x[2];
    v_ = msg.x[3];
  } else if (msg.x.size() == ConfigurationDimension()) {
    x_ = msg.x[0];
    y_ = msg.x[1];
  } else
    throw std::runtime_error("PlanarDubins3D: msg dimension is incorrect.");
}
PlanarDubins3D::PlanarDubins3D(const VectorXd &x)
    : State(), x_(0.0), y_(0.0), theta_(0.0), v_(constants::kDefaultHeight) {
  // Check dimensions.
  if (x.size() == ConfigurationDimension()) {
    x_ = x(0);
    y_ = x(1);
  } else if (x.size() == StateDimension()) {
    x_ = x(0);
    y_ = x(1);
    theta_ = x(2);
  } else {
    throw std::runtime_error("PlanarDubins3D: incorrect input dimension.");
  }
} // namespace state

// Set non-configuration dimensions to match the given config derivative.
void PlanarDubins3D::SetConfigurationDot(const VectorXd &configuration_dot) {
  if (configuration_dot.size() != ConfigurationDimension())
    throw std::runtime_error("PlanarDubins3D: invalid configuration_dot dim.");

  v_ = configuration_dot.norm();
  theta_ = std::atan2(configuration_dot(1), configuration_dot(0));
}

// Static function to sample from the configuration space associated
// with this state space. Pass in the lower and upper bounds from
// which to sample.
VectorXd PlanarDubins3D::SampleConfiguration() {
  const size_t kConfigurationSpaceDimension = 2;

  // Initialize a uniform random distribution in (0, 1).
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Extract lower/upper configuration bounds.
  const VectorXd lower_config = PlanarDubins3D::GetConfigurationLower();
  const VectorXd upper_config = PlanarDubins3D::GetConfigurationUpper();

  // Generate random sample.
  VectorXd sample(kConfigurationSpaceDimension);
  for (size_t ii = 0; ii < kConfigurationSpaceDimension; ii++)
    sample(ii) =
        lower_config(ii) + (upper_config(ii) - lower_config(ii)) * unif(rng_);

  return sample;
}

// Sample from the state space itself.
// NOTE! Sets v_ to default value.
PlanarDubins3D PlanarDubins3D::Sample() {
  // Initialize a uniform random distribution in (0, 1).
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // Generate random sample.
  const double x = lower_.x_ + (upper_.x_ - lower_.x_) * unif(rng_);
  const double y = lower_.y_ + (upper_.y_ - lower_.y_) * unif(rng_);
  const double theta =
      lower_.theta_ + (upper_.theta_ - lower_.theta_) * unif(rng_);

  return PlanarDubins3D(x, y, theta);
}

// For a given configuration, what are the corresponding positions in
// position space that the system occupies.
// NOTE! For simplicity, this is a finite set. In future, this could
// be generalized to a collection of generic obstacles.
std::vector<Vector3d> PlanarDubins3D::OccupiedPositions() const {
  return std::vector<Vector3d>({Position()});
}

// Set bounds of the state space.
void PlanarDubins3D::SetBounds(const PlanarDubins3D &lower,
                               const PlanarDubins3D &upper) {
  lower_ = lower;
  upper_ = upper;
}

// Set state space bounds from std vectors. Layout is assumed to be
// [x, y, theta].
void PlanarDubins3D::SetBounds(const std::vector<double> &lower,
                               const std::vector<double> &upper) {
  // Check dimensions.
  if (lower.size() != 3 || upper.size() != 3)
    throw std::runtime_error("PlanarDubins3D: bad bound dimension.");

  lower_.x_ = lower[0];
  lower_.y_ = lower[1];
  lower_.theta_ = lower[2];

  upper_.x_ = upper[0];
  upper_.y_ = upper[1];
  upper_.theta_ = upper[2];
}

// Convert from VectorXd. Assume State is [x, y, theta].
void PlanarDubins3D::FromVector(const VectorXd &x) {
  if (x.size() == 3) {
    x_ = x(0);
    y_ = x(1);
    theta_ = x(2);
    return;
  }

  ROS_ERROR("PlanarDubins3D: vector is of the wrong size.");
}

// Convert to VectorXd. Assume State is [x, y, theta].
VectorXd PlanarDubins3D::ToVector() const {
  VectorXd x(3);
  x(0) = x_;
  x(1) = y_;
  x(2) = theta_;

  return x;
}

// Convert from ROS message. Assume State is [x, y, theta], [x, y, theta, v], or
// configuration only.
void PlanarDubins3D::FromRos(const fastrack_msgs::State::ConstPtr &msg) {
  if (msg->x.size() == 3) {
    // Message contains state, but not v.
    x_ = msg->x[0];
    y_ = msg->x[1];
    theta_ = msg->x[2];
  } else if (msg->x.size() == 4) {
    // Message contains state and v.
    x_ = msg->x[0];
    y_ = msg->x[1];
    theta_ = msg->x[2];
    v_ = msg->x[3];
  } else if (msg->x.size() == 3) {
    // Message contains configuration only.
    x_ = msg->x[0];
    y_ = msg->x[1];
  } else
    throw std::runtime_error(
        "PlanarDubins3D: msg is neither state nor configuration.");
}

// Convert to ROS message. Format is [x, y, theta, v].
fastrack_msgs::State PlanarDubins3D::ToRos() const {
  fastrack_msgs::State msg;
  msg.x.push_back(x_);
  msg.x.push_back(y_);
  msg.x.push_back(theta_);
  msg.x.push_back(v_);

  return msg;
}

// Get bounds of state space.
const PlanarDubins3D& PlanarDubins3D::GetLower() { return lower_; }
const PlanarDubins3D& PlanarDubins3D::GetUpper() { return upper_; }

// Get bounds of configuration space.
VectorXd PlanarDubins3D::GetConfigurationLower() {
  return lower_.Configuration();
}
VectorXd PlanarDubins3D::GetConfigurationUpper() {
  return upper_.Configuration();
}

} // namespace state
} // namespace fastrack
