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
// Base struct for tracking error bound with required ROS service conversions
// (to/from ROS service response of the appropriate type (SR)).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_BOUND_TRACKING_BOUND_ROS_H
#define FASTRACK_BOUND_TRACKING_BOUND_ROS_H

#include <fastrack/utils/types.h>
#include <fastrack/bound/tracking_bound.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace fastrack {
namespace bound {

template <typename SR>
struct TrackingBoundRos : public TrackingBound {
  // Initialize from vector.
  virtual bool Initialize(const std::vector<double>& params) = 0;

  // Convert from service response type SR.
  virtual void FromRos(const SR& res) = 0;

  // Convert to service response type SR.
  virtual SR ToRos() const = 0;

  // Returns true if this tracking error bound (at the given position) overlaps
  // with different shapes.
  virtual bool OverlapsSphere(const Vector3d& p, const Vector3d& center,
                              double radius) const = 0;
  virtual bool OverlapsBox(const Vector3d& p, const Vector3d& lower,
                           const Vector3d& upper) const = 0;

  // Returns true if this tracking error bound (at the given position) is
  // contained within a box.
  virtual bool ContainedWithinBox(const Vector3d& p, const Vector3d& lower,
                                  const Vector3d& upper) const = 0;

  // Visualize.
  virtual void Visualize(const ros::Publisher& pub,
                         const std::string& frame) const = 0;
};  //\struct TrackingBoundRos

}  //\namespace bound
}  //\namespace fastrack

#endif
