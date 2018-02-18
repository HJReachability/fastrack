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
// Defines the Tracker base class, which is templated on the following types:
// -- Value function (V)
// -- [Planner] state (PS), state msg (MPS)
// -- [Tracker] state (TS), state msg (MTS), control (TC), control msg (MTC)
// -- Tracking error bound service (SB)
// -- Planner dynamics service (SP)
//
// All trackers essentially work by subscribing to state and reference topics
// and periodically (on a timer) querying the value function for the optimal
// control, then publishing that optimal control. Trackers also must provide
// services that other nodes can use to access planner dynamics and bound
// parameters.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_TRACKING_TRACKER_H
#define FASTRACK_TRACKING_TRACKER_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>

namespace fastrack {
namespace tracking {

template<typename V, typename TS, typename TC, typename MTS, typename MTC,
         typename PS, typename MPS, typename SB, typename SP>
class Tracker : private Uncopyable {
public:
  ~Tracker() {}
  explicit Tracker()
    : initialized_(false) {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback to update tracker/planner state.
  inline void TrackerStateCallback(const MTS::ConstPtr& msg) {
    tracker_x_ = TS(msg);
  }
  inline void PlannerStateCallback(const MPS::ConstPtr& msg) {
    planner_x_ = PS(msg);
  }

  // Service callbacks for tracking bound and planner parameters.
  inline bool TrackingBoundServer(
    const SB::Request::ConstPtr& req, SB::Response::ConstPtr& res) const {
    res = value_.TrackingBound().Pack();
  }
  inline bool PlannerDynamicsServer(
    const SP::Request::ConstPtr& req, SP::Response::ConstPtr& res) const {
    res = value_.PlannerDynamics().Pack();
  }

  // Timer callback. Compute the optimal control and publish.
  void TimerCallback(const ros::TimerEvent& e) const;

  // Most recent tracker/planner states.
  TS tracker_x_;
  PS planner_x_;

  // Publishers and subscribers.
  std::string tracker_state_topic_;
  std::string planner_state_topic_;
  std::string control_topic_;

  ros::Subscriber tracker_state_sub_;
  ros::Subscriber planner_state_sub_;
  ros::Publisher control_pub_;

  // Services.
  std::string bound_name_;
  std::string planner_params_name_;

  ros::ServiceServer bound_srv_;
  ros::ServiceServer planner_params_srv_;

  // Timer.
  ros::Timer timer_;

  // Value function.
  V value_;

  // Flag for whether this class has been initialized yet.
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
}; //\class Tracker

// ----------------------------- IMPLEMEMTATION ----------------------------- //

// Initialize from a ROS NodeHandle.
template<typename V, typename TS, typename TC, typename MTS, typename MTC,
         typename PS, typename MPS, typename SB, typename SP>
bool Tracker<V, TS, TC, MTS, MTC, PS, MPS, SB, SP>::
Initialize(const ros::NodeHandle& n);

// Load parameters.
template<typename V, typename TS, typename TC, typename MTS, typename MTC,
         typename PS, typename MPS, typename SB, typename SP>
bool Tracker<V, TS, TC, MTS, MTC, PS, MPS, SB, SP>::
LoadParameters(const ros::NodeHandle& n);

// Register callbacks.
template<typename V, typename TS, typename TC, typename MTS, typename MTC,
         typename PS, typename MPS, typename SB, typename SP>
bool Tracker<V, TS, TC, MTS, MTC, PS, MPS, SB, SP>::
RegisterCallbacks(const ros::NodeHandle& n);

// Timer callback. Compute the optimal control and publish.
template<typename V, typename TS, typename TC, typename MTS, typename MTC,
           typename PS, typename MPS, typename SB, typename SP>
void Tracker<V, TS, TC, MTS, MTC, PS, MPS, SB, SP>::
TimerCallback(const ros::TimerEvent& e) const;

} //\namespace tracking
} //\namespace fastrack

#endif
