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
// Node running an OMPL kinematic planner for the PositionVelocity state space
// and BallsInBox environment, with a Box tracking bound.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/planning/ompl_kinematic_planner.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/environment/balls_in_box.h>
#include <fastrack/bound/box.h>

#include <fastrack_srvs/KinematicPlannerDynamics.h>
#include <fastrack_srvs/TrackingBoundBox.h>

#include <ros/ros.h>

namespace fp = fastrack::planning;
namespace fs = fastrack::state;
namespace fb = fastrack::bound;
namespace fe = fastrack::environment;

int main(int argc, char** argv) {
  ros::init(argc, argv, "PlannerDemo");
  ros::NodeHandle n("~");

  fastrack::planning::OmplKinematicPlanner<
    fp::og::BITstar, fs::PositionVelocity, fe::BallsInBox, fb::Box,
    fastrack_srvs::TrackingBoundBox> planner;

  if (!planner.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize planner.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
