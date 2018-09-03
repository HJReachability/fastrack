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
// Kinematics are templated on state alone and inherit from dynamics.
// Kinematics are treated just like dynamics except that they operate directly
// in the configuration space of the templated state type and assume that
// the control bounds are maximum speeds in each dimension of that
// configuration space.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_KINEMATICS_H
#define FASTRACK_DYNAMICS_KINEMATICS_H

#include <fastrack/control/vector_bound_box.h>
#include <fastrack/dynamics/dynamics.h>
#include <fastrack_srvs/KinematicPlannerDynamics.h>

#include <exception>

namespace fastrack {
namespace dynamics {

using control::VectorBoundBox;

template <typename S>
class Kinematics
    : public Dynamics<S, VectorXd, VectorBoundBox,
                      fastrack_srvs::KinematicPlannerDynamics::Response> {
 public:
  explicit Kinematics()
      : Dynamics<S, VectorXd, VectorBoundBox,
                 fastrack_srvs::KinematicPlannerDynamics::Response>() {}
  explicit Kinematics(const VectorBoundBox& bound)
      : Dynamics<S, VectorXd, VectorBoundBox,
                 fastrack_srvs::KinematicPlannerDynamics::Response>(bound) {}
  explicit Kinematics(const std::vector<double>& params)
      : Dynamics<S, VectorXd, VectorBoundBox,
                 fastrack_srvs::KinematicPlannerDynamics::Response>(params) {}
  explicit Kinematics(const VectorXd& u_lower, const VectorXd& u_upper)
      : Dynamics<S, VectorXd, VectorBoundBox,
                 fastrack_srvs::KinematicPlannerDynamics::Response>(
            VectorBoundBox(u_lower, u_upper)) {}

  // Derived classes must be able to give the time derivative of state
  // as a function of current state and control.
  inline S Evaluate(const S& x, const VectorXd& u) const;

  // Since this function does not really make sense for kinematics,
  // we will throw an error here.
  inline VectorXd OptimalControl(const S& x, const S& value_gradient) const {
    throw std::runtime_error("Kinematics: OptimalControl is not implemented.");
  }

  // Convert to the appropriate service response type.
  inline fastrack_srvs::KinematicPlannerDynamics::Response ToRos() const;

  // Convert from the appropriate service response type.
  inline void FromRos(
      const fastrack_srvs::KinematicPlannerDynamics::Response& res);

  // How much time will it take us to go between two configurations if we move
  // at max velocity between them in each dimension.
  double BestPossibleTime(const S& x1, const S& x2) const;
};  //\class Kinematics

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Derived classes must be able to give the time derivative of state
// as a function of current state and control.
// NOTE! To access the member variables from Dynamics we will need to
// use the 'this' keyword (this is a consequence of our template structure).
template <typename S>
S Kinematics<S>::Evaluate(const S& x, const VectorXd& u) const {
  // Make sure dimensions agree.
  const VectorXd c = x.Configuration();
  if (c.size() != u.size()) {
    throw std::runtime_error("Kinematics: config/control spaces not equal.");
  }

  return S(u);
}

// Convert to the appropriate service response type.
template <typename S>
fastrack_srvs::KinematicPlannerDynamics::Response Kinematics<S>::ToRos() const {
  if (!this->control_bound_)
    throw std::runtime_error("Kinematics was not initialized.");

  const auto& lower_bound = this->control_bound_->Min();
  const auto& upper_bound = this->control_bound_->Max();

  fastrack_srvs::KinematicPlannerDynamics::Response res;
  for (size_t ii = 0; ii < lower_bound.size(); ii++) {
    res.min_speed.push_back(lower_bound(ii));
    res.max_speed.push_back(upper_bound(ii));
  }

  return res;
}

// Convert from the appropriate service response type.
template <typename S>
void Kinematics<S>::FromRos(
    const fastrack_srvs::KinematicPlannerDynamics::Response& res) {
  if (res.max_speed.size() != res.min_speed.size())
    throw std::runtime_error("Kinematics: invalid service response.");

  // Populate control bounds.
  VectorXd lower_bound(res.min_speed.size());
  VectorXd upper_bound(res.min_speed.size());
  for (size_t ii = 0; ii < res.min_speed.size(); ii++) {
    lower_bound(ii) = res.min_speed[ii];
    upper_bound(ii) = res.max_speed[ii];
  }

  this->control_bound_.reset(new VectorBoundBox(lower_bound, upper_bound));
}

// How much time will it take us to go between two configurations if we move
// at max velocity between them in each dimension.
template <typename S>
double Kinematics<S>::BestPossibleTime(const S& x1, const S& x2) const {
  if (!this->control_bound_)
    throw std::runtime_error("Kinematics was not initialized.");

  // Unpack into configurations.
  const VectorXd c1 = x1.Configuration();
  const VectorXd c2 = x2.Configuration();

  // Take the maximum of the times in each dimension.
  double time = -std::numeric_limits<double>::infinity();
  for (size_t ii = 0; ii < S::ConfigurationDimension(); ii++) {
    const double time_this_dim =
        (c2(ii) >= c1(ii)) ? (c2(ii) - c1(ii)) / this->control_bound_->Max()(ii)
                           : (c2(ii) - c1(ii)) / this->control_bound_->Min()(ii);
    time = std::max(time, time_this_dim);
  }

  return time;
}

}  // namespace dynamics
}  // namespace fastrack

#endif
