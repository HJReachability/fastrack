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
// Defines the MatlabValueFunction class, which derives from the base class
// ValueFunction and is templated on the tracker/planner state
// (TS/PS), tracker/planner control (TC/PC), tracker/planner dynamics (TD/PC),
// relative state (RS), and bound (B).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_VALUE_VALUE_FUNCTION_H
#define FASTRACK_VALUE_VALUE_FUNCTION_H

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/relative_state.h>
#include <fastrack/utils/types.h>
#include <fastrack/value/value_function.h>

#include <ros/ros.h>

namespace fastrack {
namespace value {

using dynamics::Dynamics;
using dynamics::RelativeDynamics;
using state::RelativeState;

template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
class MatlabValueFunction
    : public ValueFunction<TS, TC, TD, PS, PC, PD, RS, B> {
 public:
  ~MatlabValueFunction() {}
  explicit MatlabValueFunction()
      : ValueFunction<TS, TC, TD, PS, PC, PD, RS, B>() {}

  // Initialize from file. Returns whether or not loading was successful.
  // Can be used as an alternative to intialization from a NodeHandle.
  bool InitializeFromMatFile(const std::string& file_name);

  // Value and gradient at particular relative states.
  double Value(const TS& tracker_x, const PS& planner_x) const;
  RS Gradient(const TS& tracker_x, const PS& planner_x) const;

  // Priority of the optimal control at the given tracker and planner states.
  // This is a number between 0 and 1, where 1 means the final control signal
  // should be exactly the optimal control signal computed by this
  // value function.
  double Priority(const TS& tracker_x, const PS& planner_x) const;

 private:
  // Load parameters.
  bool LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    std::string file_name;
    if (!nl.getParam("file_name", file_name)) return false;

    return InitializeFromMatFile(file_name);
  }

  // Convert a (relative) state to an index into 'data_'.
  size_t StateToIndex(const VectorXd& x) const;

  // Compute the difference vector between this (relative) state and the center
  // of the nearest voxel (i.e. voxel center minus state).
  VectorXd DirectionToCenter(const VectorXd& x) const;

  // Accessor for precomputed gradient at the given state.
  VectorXd GradientAccessor(const VectorXd& x) const;

  // Compute the grid point below a given state in dimension idx.
  double LowerGridPoint(const VectorXd& punctured, size_t idx) const;

  // Recursive helper function for gradient multilinear interpolation.
  // Takes in a state and index along which to interpolate.
  VectorXd RecursiveGradientInterpolator(const VectorXd& x, size_t idx) const;

  // Number of voxels and upper/lower bounds in each dimension.
  std::vector<size_t> num_voxels_;
  std::vector<double> voxel_size_;
  std::vector<double> lower_;
  std::vector<double> upper_;

  // Value function itself is stored in row-major order.
  std::vector<double> data_;

  // Gradient information at each voxel. One list per dimension, each in the
  // same order as 'data_'.
  std::vector<std::vector<double>> gradient_;

  // Lower and upper bounds for the value function. Used for computing the
  // 'priority' of the optimal control signal.
  double priority_lower_;
  double priority_upper_;
};  //\class MatlabValueFunction

// ---------------------------- IMPLEMENTATION  ---------------------------- //

// Value at the given relative state.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
double MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::Value(
    const TS& tracker_x, const PS& planner_x) const {
  const VectorXd relative_x = RS(tracker_x, planner_x).ToVector();

  // Get distance from voxel center in each dimension.
  const VectorXd center_distance = DistanceToCenter(relative_x);

  // Interpolate.
  const double nn_value = data_[StateToIndex(relative_x)];
  double approx_value = nn_value;

  VectorXd neighbor = relative_x;
  for (size_t ii = 0; ii < relative_x.size(); ii++) {
    // Get neighboring value.
    if (center_distance(ii) >= 0.0)
      neighbor(ii) += voxel_size_[ii];
    else
      neighbor(ii) -= voxel_size_[ii];

    const double neighbor_value = data_[StateToIndex(neighbor)];
    neighbor(ii) = relative_x(ii);

    // Compute forward difference.
    const double slope = (center_distance(ii) >= 0.0)
                             ? (neighbor_value - nn_value) / voxel_size_[ii]
                             : (nn_value - neighbor_value) / voxel_size_[ii];

    // Add to the Taylor approximation.
    approx_value += slope * center_distance(ii);
  }

  return approx_value;
}

// Gradient at the given relative state.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
RS MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::Gradient(
    const TS& tracker_x, const PS& planner_x) const {
  const VectorXd relative_x = RS(tracker_x, planner_x).ToVector();
  return RecursiveGradientInterpolator(relative_x, 0);
}

// Priority of the optimal control at the given tracker and planner states.
// This is a number between 0 and 1, where 1 means the final control signal
// should be exactly the optimal control signal computed by this
// value function.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
double MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::Priority(
    const TS& tracker_x, const PS& planner_x) const {
  const double value = Value(tracker_x, planner_x);

  if (value < priority_lower_) return 0.0;
  if (value > priority_upper_) return 1.0;

  return (value - priority_lower_) / (priority_upper_ - priority_lower_);
}

// Convert a (relative) state to an index into 'data_'.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
size_t MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::StateToIndex(
    const VectorXd& x) {
  // Quantize each dimension of the state.
  std::vector<size_t> quantized;
  for (size_t ii = 0; ii < x.size(); ii++) {
    if (x(ii) < lower_[ii]) {
      ROS_WARN("%s: State is too small in dimension %zu.", name_.c_str(), ii);
      quantized.push_back(0);
    } else if (x(ii) > upper_[ii]) {
      ROS_WARN("%s: State is too large in dimension %zu.", name_.c_str(), ii);
      quantized.push_back(num_voxels_[ii] - 1);
    } else {
      // In bounds, so quantize. This works because of 0-indexing and casting.
      quantized.push_back(
          static_cast<size_t>((x(ii) - lower_[ii]) / voxel_size_[ii]));
    }
  }

  // Convert to row-major order.
  size_t idx = quantized[0];
  for (size_t ii = 1; ii < quantized.size(); ii++) {
    idx *= num_voxels_[ii];
    idx += quantized[ii];
  }

  return idx;
}

// Accessor for precomputed gradient at the given state.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
VectorXd MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::GradientAccessor(
    const VectorXd& x) {
  // Convert to index and read gradient one dimension at a time.
  const size_t idx = StateToIndex(x);

  VectorXd gradient(x.size());
  for (size_t ii = 0; ii < gradient.size(); ii++)
    gradient(ii) = gradient_[ii][idx];

  return gradient;
}

// Compute the grid point below a given state in dimension idx.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
double MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::LowerGridPoint(
    const VectorXd& x, size_t idx) const {
  // Get center of nearest voxel.
  const double center =
      0.5 * voxel_size_[idx] + lower_[idx] +
      voxel_size_[idx] * std::floor((x(idx) - lower_[idx]) / voxel_size_[idx]);

  // Check if center is above us. If so, the lower bound is the voxel below.
  return (center > x(idx)) ? center - voxel_size_[idx] : center;
}

// Recursive helper function for gradient multilinear interpolation.
// Takes in a state and index along which to interpolate.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
VectorXd MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::
    RecursiveGradientInterpolator(const VectorXd& x, size_t idx) {
  // Assume x's entries prior to idx are equal to the upper/lower bounds of
  // the voxel containing x.
  // Begin by computing the lower and upper bounds of the voxel containing x
  // in dimension idx.
  const double lower = LowerGridPoint(x, idx);
  const double upper = lower + voxel_size_[idx];

  // Compute the fractional distance between lower and upper.
  const double fractional_dist = (x(idx) - lower) / voxel_size_[idx];

  // Split x along dimension idx.
  VectorXd x_lower = x;
  x_lower(idx) = lower;

  VectorXd x_upper = x;
  x_upper(idx) = upper;

  // Base case.
  if (idx == x.size() - 1) {
    return GradientAccessor(x_upper) * fractional_dist +
           GradientAccessor(x_lower) * (1.0 - fractional_dist);
  }

  // Recursive step.
  return RecursiveGradientInterpolator(x_upper, idx + 1) * fractional_dist +
         RecursiveGradientInterpolator(x_lower, idx + 1) *
             (1.0 - fractional_dist);
}

// Initialize from file. Returns whether or not loading was successful.
// Can be used as an alternative to intialization from a NodeHandle.
template <typename TS, typename TC, typename TD, typename PS, typename PC,
          typename PD, typename RS, typename B>
bool MatlabValueFunction<TS, TC, TD, PS, PC, PD, RS, B>::InitializeFromMatFile(
    const std::string& file_name) {
  // TODO(dfk)!
  return true;
}

}  // namespace value
}  // namespace fastrack

#endif
