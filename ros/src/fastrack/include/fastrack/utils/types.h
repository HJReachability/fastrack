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
// Custom types.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_UTILS_TYPES_H
#define FASTRACK_UTILS_TYPES_H

// ------------------------------- INCLUDES -------------------------------- //

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <math.h>
#include <memory>
#include <random>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

// ------------------------------- CONSTANTS -------------------------------- //

namespace fastrack {
namespace constants {
// Acceleration due to gravity.
static constexpr double G = 9.81;

// Small number for use in approximate equality checking.
static constexpr double kEpsilon = 1e-4;

// Double precision infinity.
static constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Default speed of 1 m/s.
static constexpr double kDefaultSpeed = 1.0;

// Default height (e.g. for planar state space models).
static constexpr double kDefaultHeight = 1.0;
} // namespace constants

// Empty struct for setting unused/unimplemented template args.
struct Empty {};
} // namespace fastrack

// ---------------------------- SIMPLE FUNCTIONS ---------------------------- //

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// ------------------------ THIRD PARTY TYPEDEFS ---------------------------- //

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

#endif
