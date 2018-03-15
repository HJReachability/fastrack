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
// BallsInBox is derived from the Environment base class. This class models
// obstacles as spheres to provide a simple demo.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_ENVIRONMENT_BALLS_IN_BOX_H
#define FASTRACK_ENVIRONMENT_BALLS_IN_BOX_H

#include <fastrack/environment/environment.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace fastrack {
namespace environment {

class BallsInBox : public Environment {
public:
  ~BallsInBox() {}
  explicit BallsInBox()
    : Environment() {}

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided position is a valid collision-free configuration.
  // Provide a separate collision check for each type of tracking error bound.
  bool IsValid(const Vector3d& position, const Box& bound) const;

  // Derived classes must have some sort of visualization through RViz.
  void Visualize(const ros::Publisher& pub, const std::string& frame) const;

private:
  // Load parameters. This may be overridden by derived classes if needed
  // (they should still call this one via Environment::LoadParameters).
  bool LoadParameters(const ros::NodeHandle& n);

  // Generate random obstacles.
  void GenerateObstacles(size_t num, double min_radius, double max_radius,
                         unsigned int seed=0);

  // Obstacle centers and radii.
  std::vector<Vector3d> centers_;
  std::vector<double> radii_;
}; //\class Environment

} //\namespace environment
} //\namespace fastrack

#endif
