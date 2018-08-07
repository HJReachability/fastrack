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

#include <fastrack/planning/planar_dubins_planner.h>

namespace fastrack {
namespace planning {

// Generate a sub-plan that connects two states and is dynamically feasible
// (but not necessarily recursively feasible).
Trajectory<PlanarDubins3D> PlanarDubinsPlanner::SubPlan(
    const PlanarDubins3D& start, const PlanarDubins3D& goal,
    double start_time = 0.0) const {
  // Create an OMPL state space.
  auto space = std::make_shared<ob::DubinsStateSpace>(dynamics_.TurningRadius());

  // Parse start/goal states into OMPL format.
  const auto& ompl_start = ToOmplState(start, space);
  const auto& ompl_goal = ToOmplState(goal, space);

  // Set up state space bounds.
  constexpr size_t kNumRealVectorBoundDimensions = 2;
  ob::RealVectorBounds bounds(kNumRealVectorBoundDimensions);

  bounds.setLow(0, PlanarDubins3D::lower_.X());
  bounds.setLow(1, PlanarDubins3D::lower_.Y());
  bounds.setHigh(0, PlanarDubins3D::upper_.X());
  bounds.setHigh(1, PlanarDubins3D::upper_.Y());
  space->setBounds(bounds);

  // Set up OMPL solver.
  og::SimpleSetup ompl_setup(space);
  ompl_setup.setStateValidityChecker([&](const ob::State* state) {
    return env_.AreValid(FromOmplState(state).OccupiedPositions(), bound_);
  });

  ompl_setup.setStartAndGoalStates(ompl_start, ompl_goal);

  // Solve.
  if (!ompl_setup.solve(max_runtime_)) {
    ROS_WARN("%s: Could not compute a valid solution.");
    return Trajectory<PlanarDubins3D>();
  }

  // Unpack the solution, upsample to include roughly the states used for
  // validity checking at planning time, and assign timesteps.
  auto path = ompl_setup.getSolutionPath();
  path.interpolate();

  std::vector<PlanarDubins3D> states;
  std::vector<double> times;

  double time = start_time;
  for (size_t ii = 0; ii < path.getStateCount(); ii++) {
    const auto state = FromOmplState(path.getState(ii));

    if (ii > 0)
      time +=
          (state.Position() - states.back().Position()).norm() / dynamics_.V();

    states.emplace_back(state);
    times.emplace_back(time);
  }

  return Trajectory<PlanarDubins3D>(states, times);
}

// Convert OMPL state to/from PlanarDubins3D.
PlanarDubins3D PlanarDubinsPlanner::FromOmplState(const ob::State* ompl_state) {
  // Catch null state.
  if (!ompl_state)
    throw std::runtime_error("PlanarDubinsPlanner: null OMPL state.");

  // Cast to proper type.
  const ob::SE2StateSpace::StateType* cast_state =
      static_cast<const ob::SE2StateSpace::StateType*>(ompl_state);

  // Populate PlanarDubins3D state.
  return PlanarDubins3D(cast_state->values[0], cast_state->values[1],
                        cast_state[2], dynamics_.V());
}

ob::ScopedState<ob::SE2StateSpace> PlanarDubinsPlanner::ToOmplState(
    const PlanarDubins3D& state,
    const std::shared_ptr<ob::SE2StateSpace>& space) {
  ob::ScopedState<ob::SE2StateSpace> ompl_state(space);

  // Populate each field.
  ompl_state[0] = state.X();
  ompl_state[1] = state.Y();
  ompl_state[2] = state.Theta();

  return ompl_state;
}

}  //\namespace planning
}  //\namespace fastrack
