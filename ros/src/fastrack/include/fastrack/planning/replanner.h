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
// Defines the Replanner class, which listens for replanning requests,
// makes the appropriate service calls, and re-sends the service response
// back on the appropriate topic.
//
// NOTE! This class exists because the Planner interface is service-based,
// rather than pub/sub-based, and the PlannerManager cannot afford to wait for
// the service calls to finish (i.e. for the planner to compute) before
// sending the next planner reference state.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_REPLANNER_H
#define FASTRACK_PLANNING_REPLANNER_H

#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <fastrack_msgs/ReplanRequest.h>
#include <fastrack_msgs/Trajectory.h>
#include <fastrack_srvs/Replan.h>
#include <fastrack_srvs/ReplanRequest.h>
#include <fastrack_srvs/ReplanResponse.h>

#include <ros/ros.h>

namespace fastrack {
namespace planning {

using trajectory::Trajectory;

class Replanner : private Uncopyable {
public:
  ~Replanner() {}
  explicit Replanner()
    : initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing replanning requests.
  void ReplanRequestCallback(
    const fastrack_msgs::ReplanRequest::ConstPtr& msg);

  // Publishers/subscribers and related topics.
  ros::Publisher traj_pub_;
  ros::Subscriber replan_request_sub_;

  std::string replan_request_topic_;
  std::string traj_topic_;

  // Services.
  ros::ServiceClient replan_srv_;
  std::string replan_srv_name_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

} //\namespace planning
} //\namespace fastrack

#endif
