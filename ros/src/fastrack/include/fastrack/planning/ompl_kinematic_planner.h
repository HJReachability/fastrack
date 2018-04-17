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
// OMPL kinematic planner wrapper, templated on the state type and the specific
// type of OMPL planner to be used under the hood.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_OMPL_KINEMATIC_PLANNER_H
#define FASTRACK_PLANNING_OMPL_KINEMATIC_PLANNER_H

#include <fastrack/planning/kinematic_planner.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

namespace fastrack {
namespace planning {

namespace ob = ompl::base;
namespace og = ompl::geometric;

template<typename P, typename S, typename E, typename B, typename SB>
class OmplKinematicPlanner : public KinematicPlanner<S, E, B, SB> {
public:
  ~OmplKinematicPlanner() {}
  explicit OmplKinematicPlanner()
    : KinematicPlanner<S, E, B, SB>() {
    // Set OMPL log level.
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
  }

private:
  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  // NOTE! The states in the output trajectory are essentially configurations.
  Trajectory<S> Plan(const S& start, const S& goal, double start_time=0.0) const;

  // Convert between OMPL states and configurations.
  S FromOmplState(const ob::State* state) const;
}; //\class KinematicPlanner

// ---------------------------- IMPLEMENTATION ------------------------------ //

// Convert between OMPL states and configurations.
template<typename P, typename S, typename E, typename B, typename SB>
S OmplKinematicPlanner<P, S, E, B, SB>::FromOmplState(const ob::State* state) const {
  // Catch null state.
  if (!state)
    throw std::runtime_error("OmplKinematicPlanner: null OMPL state.");

  // Cast state to proper type.
  const ob::RealVectorStateSpace::StateType* cast_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  // Populate configuration.
  VectorXd config(S::ConfigurationDimension());
  for (size_t ii = 0; ii < S::ConfigurationDimension(); ii++)
    config(ii) = cast_state->values[ii];

  return S(config);
}

// Plan a trajectory from the given start to goal states starting
// at the given time.
template<typename P, typename S, typename E, typename B, typename SB>
Trajectory<S> OmplKinematicPlanner<P, S, E, B, SB>::
Plan(const S& start, const S& goal, double start_time) const {
  // Unpack start and goal configurations.
  const VectorXd start_config = start.Configuration();
  const VectorXd goal_config = goal.Configuration();

  // Check that both start and stop are in bounds.
  if (!this->env_.AreValid(start.OccupiedPositions(), this->bound_)) {
    ROS_WARN_THROTTLE(1.0, "Start point was in collision or out of bounds.");
    return Trajectory<S>();
  }

  if (!this->env_.AreValid(goal.OccupiedPositions(), this->bound_)) {
    ROS_WARN_THROTTLE(1.0, "Goal point was in collision or out of bounds.");
    return Trajectory<S>();
  }

  // Create the OMPL state space corresponding to this environment.
  auto ompl_space(
    std::make_shared<ob::RealVectorStateSpace>(S::ConfigurationDimension()));

  // Set bounds for the environment.
  const VectorXd lower = S::GetConfigurationLower();
  const VectorXd upper = S::GetConfigurationUpper();

  ob::RealVectorBounds ompl_bounds(S::ConfigurationDimension());
  for (size_t ii = 0; ii < S::ConfigurationDimension(); ii++) {
    ompl_bounds.setLow(ii, lower(ii));
    ompl_bounds.setHigh(ii, upper(ii));
  }

  ompl_space->setBounds(ompl_bounds);

  // Create a SimpleSetup instance and set the state validity checker function.
  og::SimpleSetup ompl_setup(ompl_space);
  ompl_setup.setStateValidityChecker([&](const ob::State* state) {
      return this->env_.AreValid(FromOmplState(state).OccupiedPositions(), this->bound_);
    });

  // Set the start and goal states.
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ompl_space);
  ob::ScopedState<ob::RealVectorStateSpace> ompl_goal(ompl_space);
  for (size_t ii = 0; ii < S::ConfigurationDimension(); ii++) {
    ompl_start[ii] = start_config(ii);
    ompl_goal[ii] = goal_config(ii);
  }

  ompl_setup.setStartAndGoalStates(ompl_start, ompl_goal);

  // Set the planner.
  ob::PlannerPtr ompl_planner(new P(ompl_setup.getSpaceInformation()));
  ompl_setup.setPlanner(ompl_planner);

  // Solve. Parameter is the amount of time (in seconds) used by the solver.
  const ob::PlannerStatus solved = ompl_setup.solve(this->max_runtime_);

  if (!solved) {
    ROS_WARN("OMPL Planner could not compute a solution.");
    return Trajectory<S>();
  }

  // Unpack the solution and assign timestamps.
  const og::PathGeometric& solution = ompl_setup.getSolutionPath();

  // Populate the Trajectory with states and time stamps.
  // NOTE! These states are essentially just configurations.
  std::vector<S> states;
  std::vector<double> times;

  double time = start_time;
  for (size_t ii = 0; ii < solution.getStateCount(); ii++) {
    const S state = FromOmplState(solution.getState(ii));

    // Increment time by the duration it takes us to get from the previous
    // configuration to this one.
    if (ii > 0)
      time += this->dynamics_.BestPossibleTime(states.back(), state);

    times.push_back(time);
    states.push_back(state);
  }

  return Trajectory<S>(states, times);
}

} //\namespace planning
} //\namespace fastrack

#endif
