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
///
// PositionVelocityState estimator node. Sets a recurring timer and every time
// it rings, this node will query tf for the transform between the specified
// robot frame and the fixed frame, merge it with the previous state estimate,
// and publish the new estimate.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_STATE_ESTIMATOR_POSITION_VELOCITY_STATE_ESTIMATOR_H
#define CRAZYFLIE_STATE_ESTIMATOR_POSITION_VELOCITY_STATE_ESTIMATOR_H

#include <crazyflie_state_estimator/state_estimator.h>
#include <crazyflie_msgs/PositionVelocityStateStamped.h>

#include <ros/ros.h>
#include <math.h>

class PositionVelocityStateEstimator : public StateEstimator {
public:
  virtual ~PositionVelocityStateEstimator() {}
  explicit PositionVelocityStateEstimator()
    : StateEstimator() {}

private:
  // Register callbacks.
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Merge a pose measured at the given time (specified by translation
  // and euler angles) into the current state estimate.
  void Update(const Vector3d& translation, const Vector3d& euler,
              const ros::Time& stamp);
}; //\class PositionVelocityStateEstimator

#endif
