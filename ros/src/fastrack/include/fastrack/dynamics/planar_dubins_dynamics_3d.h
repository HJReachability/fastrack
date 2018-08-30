/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
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
// Defines the PlanarDubinsDynamics3D class, which uses the PlanarDubins3D
// state type.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_DYNAMICS_PLANAR_DUBINS_DYNAMICS_3D_H
#define FASTRACK_DYNAMICS_PLANAR_DUBINS_DYNAMICS_3D_H

#include <fastrack/control/scalar_bound_interval.h>
#include <fastrack/dynamics/dynamics.h>
#include <fastrack/state/planar_dubins_3d.h>
#include <fastrack_srvs/PlanarDubinsPlannerDynamics.h>
#include <fastrack_srvs/PlanarDubinsPlannerDynamicsResponse.h>

#include <math.h>

namespace fastrack {
namespace dynamics {

using state::PlanarDubins3D;
using control::ScalarBoundInterval;

class PlanarDubinsDynamics3D
    : public Dynamics<PlanarDubins3D, double, ScalarBoundInterval,
                      fastrack_srvs::PlanarDubinsPlannerDynamics::Response> {
 public:
  ~PlanarDubinsDynamics3D() {}
  explicit PlanarDubinsDynamics3D()
      : Dynamics<PlanarDubins3D, double, ScalarBoundInterval,
                 fastrack_srvs::PlanarDubinsPlannerDynamics::Response>() {}
  explicit PlanarDubinsDynamics3D(const ScalarBoundInterval& bound)
      : Dynamics<PlanarDubins3D, double, ScalarBoundInterval,
                 fastrack_srvs::PlanarDubinsPlannerDynamics::Response>(bound) {}
  explicit PlanarDubinsDynamics3D(const std::vector<double>& params)
      : Dynamics<PlanarDubins3D, double, ScalarBoundInterval,
                 fastrack_srvs::PlanarDubinsPlannerDynamics::Response>(params) {
  }
  explicit PlanarDubinsDynamics3D(const double& u_lower, const double& u_upper)
      : Dynamics<PlanarDubins3D, double, ScalarBoundInterval,
                 fastrack_srvs::PlanarDubinsPlannerDynamics::Response>(
            ScalarBoundInterval(u_lower, u_upper)) {
    // Make sure interval is symmetric.
    constexpr double kSmallNumber = 1e-8;
    if (std::abs(control_bound_->Max() + control_bound_->Min() > kSmallNumber))
      throw std::runtime_error("Non-symmetric control bound.");
  }

  // Accessors.
  double V() const { return v_; }
  double MaxOmega() const { return control_bound_->Max(); }

  // Compute turning radius.
  double TurningRadius() const { return V() / MaxOmega(); }

  // Derived classes must be able to give the time derivative of state
  // as a function of current state and control.
  inline PlanarDubins3D Evaluate(const PlanarDubins3D& x,
                                 const double& u) const {
    const PlanarDubins3D x_dot(v_ * std::cos(x.Theta()),
                               v_ * std::sin(x.Theta()), u);
    return x_dot;
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the specified state.
  // In this case (linear dynamics), the state is irrelevant given the
  // gradient of the value function at that state.
  inline double OptimalControl(const PlanarDubins3D& x,
                               const PlanarDubins3D& value_gradient) const {
    throw std::runtime_error(
        "PlanarDubinsDynamics3D: OptimalControl is unimplemented.");
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Convert to the appropriate service response type.
  inline fastrack_srvs::PlanarDubinsPlannerDynamics::Response ToRos() const {
    if (!control_bound_)
      throw std::runtime_error("PlanarDubinsDynamics3D was not initialized.");

    fastrack_srvs::PlanarDubinsPlannerDynamics::Response res;
    res.speed = v_;
    res.max_yaw_rate = control_bound_->Max();

    return res;
  }

  // Convert from the appropriate service response type.
  inline void FromRos(
      const fastrack_srvs::PlanarDubinsPlannerDynamics::Response& res) {
    v_ = res.speed;
    control_bound_.reset(
        new ScalarBoundInterval(-res.max_yaw_rate, res.max_yaw_rate));
  }

 private:
  double v_;
};  //\class QuadrotorDecoupled6D

}  // namespace dynamics
}  // namespace fastrack

#endif
