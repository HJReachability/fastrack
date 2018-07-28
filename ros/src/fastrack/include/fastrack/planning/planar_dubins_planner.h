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
 *          Jaime Fisac            ( jfisac@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// OMPL-based Dubins car planner, using GraphDynamicPlanner high level logic.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_PLANAR_DUBINS_PLANNER_H
#define FASTRACK_PLANNING_PLANAR_DUBINS_PLANNER_H

#include <fastrack/planning/graph_dynamic_planner.h>
#include <fastrack/utils/types.h>

namespace fastrack {
namespace planning {

using environment::BallsInBoxOccupancyMap;
using bound::Box;
using dynamics::PlanarDubinsDynamics3D;
using state::PlanarDubins3D;

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlanarDubinsPlanner
    : public GraphDynamicPlanner<PlanarDubins3D, BallsInBoxOccupancyMap,
                                 PlanarDubinsDynamics3D,
                                 fastrack_srvs::PlanarDubinsPlannerDynamics,
                                 Box, fastrack_srvs::TrackingBoundBox> {
 public:
  ~PlanarDubinsPlanner() {}
  explicit PlanarDubinsPlanner() {}

 private:
  // Generate a sub-plan that connects two states and is dynamically feasible
  // (but not necessarily recursively feasible).
  Trajectory<PlanarDubins3D> SubPlan(const PlanarDubins3D& start,
                                     const PlanarDubins3D& goal,
                                     double start_time = 0.0) const;

  // Convert OMPL state to/from PlanarDubins3D.
  PlanarDubins3D FromOMPL(const ob::State* ompl_state);
  static ob::ScopedState<ob::SE2StateSpace> ToOMPL(
      const PlanarDubins3D& state,
      const std::shared_ptr<ob::SE2StateSpace>& space);

};  //\class PlanarDubinsPlanner

}  //\namespace planning
}  //\namespace fastrack

#endif
