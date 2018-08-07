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
// Class to specify a box constraint on a vector-valued control variable.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CONTROL_VECTOR_BOUND_BOX_H
#define FASTRACK_CONTROL_VECTOR_BOUND_BOX_H

#include <fastrack/control/control_bound.h>
#include <fastrack/control/scalar_bound_interval.h>

namespace fastrack {
namespace control {

class VectorBoundBox : public ControlBound<VectorXd> {
 public:
  ~VectorBoundBox() {}
  explicit VectorBoundBox(const VectorXd &min, const VectorXd &max)
      : min_(min), max_(max) {
    if (min_.size() != max_.size())
      throw std::runtime_error("Inconsistent bound dimensions.");
  }

  // Assume 'params' is laid out such that the first half of the parameters
  // form the 'min_' and the second form the 'max_'.
  explicit VectorBoundBox(const std::vector<double> &params) {
    if (!params.size() || params.size() & 1)
      throw std::runtime_error("Incorrect number of parameters.");

    const size_t dimension = params.size() >> 1;

    min_.resize(dimension);
    for (size_t ii = 0; ii < dimension; ii++) min_(ii) = params[ii];

    max_.resize(dimension);
    for (size_t ii = 0; ii < dimension; ii++) max_(ii) = params[dimension + ii];
  }

  // Accessors.
  inline const VectorXd &Min() const { return min_; }
  inline const VectorXd &Max() const { return max_; }

  // Derived classes must be able to check whether a query is inside the bound.
  inline bool Contains(const VectorXd &query) const {
    if (min_.size() != query.size()) {
      ROS_ERROR("VectorBoundBox: incorrect query dimension.");
      return false;
    }

    for (size_t ii = 0; ii < min_.size(); ii++) {
      if (min_(ii) > query(ii) || query(ii) > max_(ii)) return false;
    }

    return true;
  }

  // Derived classes must be able to compute the projection of a vector
  // (represented as the templated type) onto the surface of the bound.
  // NOTE: We will treat this vector as emanating from the natural origin
  // of the bound so that it constitutes a meaningful direction with respect
  // to that origin.
  inline VectorXd ProjectToSurface(const VectorXd &query) const {
    if (min_.size() != query.size()) {
      ROS_ERROR("VectorBoundBox: incorrect query dimension.");
      return VectorXd::Zero(min_.size());
    }

    VectorXd projection(min_.size());
    for (size_t ii = 0; ii < min_.size(); ii++)
      projection(ii) = (query(ii) >= 0.0) ? max_(ii) : min_(ii);

    return projection;
  }

 private:
  // Lower and upper bounds..
  VectorXd min_, max_;
};  //\class ControlBound

}  // namespace control
}  // namespace fastrack

#endif
