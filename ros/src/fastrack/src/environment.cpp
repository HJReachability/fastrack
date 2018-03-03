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
// Base class for all environment models, providing separate collision check
// functions for each type of tracking error bound. All environments are
// boxes in 3D space.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/environment/environment.h>

namespace fastrack {
namespace environment {

// Initialize from a ROS NodeHandle.
bool Environment::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Environment");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Provide auxiliary validity checkers for sets of positions.
bool Environment::IsValid(
  const std::vector<Vector3d>& positions, const Box& bound) const {
  // Return Boolean AND of all IsValid calls.
  for (const auto& p : positions) {
    if (!IsValid(p, bound))
      return false;
  }

  return true;
}

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via Environment::LoadParameters).
bool Environment::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Upper and lower bounds of the environment.
  if (!nl.getParam("env/upper/x", upper_(0))) return false;
  if (!nl.getParam("env/upper/y", upper_(1))) return false;
  if (!nl.getParam("env/upper/z", upper_(2))) return false;

  if (!nl.getParam("env/lower/x", lower_(0))) return false;
  if (!nl.getParam("env/lower/y", lower_(1))) return false;
  if (!nl.getParam("env/lower/z", lower_(2))) return false;

  return true;
}

} //\namespace environment
} //\namespace fastrack

