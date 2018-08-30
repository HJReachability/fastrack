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
// Node running a Tracker based on the MatlabValueFunction class.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/bound/box.h>
#include <fastrack/control/quadrotor_control.h>
#include <fastrack/dynamics/planar_dubins_dynamics_3d.h>
#include <fastrack/dynamics/quadrotor_decoupled_6d.h>
#include <fastrack/dynamics/quadrotor_decoupled_6d_rel_planar_dubins_3d.h>
#include <fastrack/state/planar_dubins_3d.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/tracking/tracker.h>
#include <fastrack/utils/types.h>
#include <fastrack/value/matlab_value_function.h>

#include <fastrack_srvs/KinematicPlannerDynamics.h>
#include <fastrack_srvs/TrackingBoundBox.h>

#include <ros/ros.h>

namespace ft = fastrack::tracking;
namespace fs = fastrack::state;
namespace fb = fastrack::bound;
namespace fc = fastrack::control;
namespace fd = fastrack::dynamics;
namespace fv = fastrack::value;

using CustomValueFunction = fv::MatlabValueFunction<
    fs::PositionVelocity, fc::QuadrotorControl,
    fd::QuadrotorDecoupled6D<fc::QuadrotorControlBoundCylinder>,
    fs::PlanarDubins3D, double, fd::PlanarDubinsDynamics3D,
    fs::PositionVelocityRelPlanarDubins3D,
    fd::QuadrotorDecoupled6DRelPlanarDubins3D, fb::Box>;

int main(int argc, char** argv) {
  ros::init(argc, argv, "TrackerDemo");
  ros::NodeHandle n("~");

  ft::Tracker<CustomValueFunction, fs::PositionVelocity, fc::QuadrotorControl,
              fs::PlanarDubins3D, fastrack_srvs::TrackingBoundBox,
              fastrack_srvs::PlanarDubinsPlannerDynamics>
      tracker;

  if (!tracker.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize tracker.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
