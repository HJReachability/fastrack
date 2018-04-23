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
// Base class for all state types. All states must be able to output a position
// in 3D space and an arbitrary-dimensional configuration. This configuration
// will be used for geometric planning.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_STATE_STATE_H
#define FASTRACK_STATE_STATE_H

#include <fastrack/utils/types.h>
#include <fastrack_msgs/State.h>

namespace fastrack {
namespace state {

class State {
public:
  virtual ~State() {}

  // Accessors. All states must be able to output a position in 3D space
  // and an arbitrary-dimensional configuration. This configuration will
  // be used for geometric planning.
  virtual double X() const = 0;
  virtual double Y() const = 0;
  virtual double Z() const = 0;
  virtual Vector3d Position() const = 0;
  virtual VectorXd Configuration() const = 0;

  // What are the positions that the system occupies at the current state.
  // NOTE! For simplicity, this is a finite set. In future, this could
  // be generalized to a collection of generic obstacles.
  virtual std::vector<Vector3d> OccupiedPositions() const = 0;

  // Convert from/to VectorXd.
  virtual void FromVector(const VectorXd& x) = 0;
  virtual VectorXd ToVector() const = 0;

  // Convert from/to ROS message.
  virtual void FromRos(const fastrack_msgs::State::ConstPtr& msg) = 0;
  virtual fastrack_msgs::State ToRos() const = 0;

  // Re-seed the random engine.
  static inline void Seed(unsigned int seed) { rng_.seed(seed); }

protected:
  explicit State() {}

  // Random number generator shared across all instances of states.
  static std::random_device rd_;
  static std::default_random_engine rng_;
}; //\class State

} //\namespace state
} //\namespace fastrack

#endif
