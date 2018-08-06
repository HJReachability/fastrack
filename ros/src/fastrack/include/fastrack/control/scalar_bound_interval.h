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
// Class to specify an interval constraint on a scalar control variable.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CONTROL_SCALAR_BOUND_INTERVAL_H
#define FASTRACK_CONTROL_SCALAR_BOUND_INTERVAL_H

#include <fastrack/control/control_bound.h>

namespace fastrack {
namespace control {

class ScalarBoundInterval : public ControlBound<double> {
 public:
  ~ScalarBoundInterval() {}
  explicit ScalarBoundInterval(double min, double max) : min_(min), max_(max) {}

  // Assume 'params' is [min, max].
  explicit ScalarBoundInterval(const std::vector<double> &params)
      : min_(params[0]), max_(params_[1]) {}

  // Derived classes must be able to check whether a query is inside the bound.
  inline bool Contains(const double &query) const {
    return min_ <= query && query <= max_;
  }

  // Derived classes must be able to compute the projection of a vector
  // (represented as the templated type) onto the surface of the bound.
  // NOTE: We will treat this vector as emanating from the natural origin
  // of the bound so that it constitutes a meaningful direction with respect
  // to that origin.
  inline double ProjectToSurface(const double &query) const {
    return (query >= 0.0) ? max_ : min_;
  }

 private:
  // Min and max interval values.
  const double min_, max_;
};  //\class ControlBound

}  // namespace control
}  // namespace fastrack

#endif
