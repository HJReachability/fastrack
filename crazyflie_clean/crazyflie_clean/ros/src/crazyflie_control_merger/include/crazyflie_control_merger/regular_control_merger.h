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
// Class to merge control messages from two different controllers into
// a single ControlStamped message.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_CONTROL_MERGER_REGULAR_CONTROL_MERGER_H
#define CRAZYFLIE_CONTROL_MERGER_REGULAR_CONTROL_MERGER_H

#include <crazyflie_control_merger/control_merger.h>
#include <crazyflie_msgs/PrioritizedControlStamped.h>
#include <crazyflie_msgs/PrioritizedControl.h>
#include <crazyflie_msgs/ControlStamped.h>
#include <crazyflie_msgs/Control.h>

#include <ros/ros.h>
#include <math.h>

namespace crazyflie_control_merger {

class RegularControlMerger : public ControlMerger {
public:
  virtual ~RegularControlMerger() {}
  explicit RegularControlMerger()
    : ControlMerger() {}

private:
  // Register callbacks.
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Process an incoming prioritized control signal.
  void PrioritizedControlCallback(
    const crazyflie_msgs::PrioritizedControlStamped::ConstPtr& msg);

  // Every derived class must implement a function to merge and publish control.
  void PublishMergedControl() const;

  // Most recent priortized control signal.
  crazyflie_msgs::PrioritizedControl prioritized_control_;
}; //\class NoYawMerger

} //\crazyflie_control_merger

#endif
