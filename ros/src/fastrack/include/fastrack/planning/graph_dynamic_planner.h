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
// Base class for all graph-based dynamic planners. These planners are all
// guaranteed to generate recursively feasible trajectories constructed
// using sampling-based logic.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_GRAPH_DYNAMIC_PLANNER_H
#define FASTRACK_PLANNING_GRAPH_DYNAMIC_PLANNER_H

#include <fastrack/planning/planner.h>
#include <fastrack/planning/searchable_set.h>
#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/types.h>

namespace fastrack {
namespace planning {

using dynamics::Dynamics;

template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
class GraphDynamicPlanner : public Planner< S, E, D, SD, B, SB > {
public:
  virtual ~GraphDynamicPlanner() {}

protected:
  explicit GraphDynamicPlanner()
    : Planner<S, E, D, SD, B, SB>() {}

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  inline Trajectory<S> Plan(
    const S& start, const S& goal, double start_time=0.0) const {
    return RecursivePlan(
      start, SearchableSet< Node<S>, S >(goal), start_time, true);
  }

  // Recursive version of Plan() that plans outbound and return trajectories.
  // High level recursive feasibility logic is here.
  Trajectory<S> RecursivePlan(
    const S& start, const S& goal, double start_time, bool outbound,
    SearchableSet< Node<S>, S >& viable_states,
    SearchableSet< Node<S>, S >& undecided_states);

  // Generate a sub-plan that connects two states and is dynamically feasible
  // (but not necessarily recursively feasible).
  virtual Trajectory<S> SubPlan(
    const S& start, const S& goal, double start_time=0.0) const = 0;

  // Node in implicit planning graph, templated on state type.
  // NOTE! To avoid memory leaks, Nodes are constructed using a static
  // factory method that returns a shared pointer.
  template<typename S>
  struct Node<S> {
    S state;
    Node<S>::ConstPtr child;
    Trajectory<S> traj;

    // Typedefs.
    typedef std::shared_ptr< Node<S> > Ptr;
    typedef std::shared_ptr< const Node<S> > ConstPtr;

    // Factory methods.
    static Ptr Create();
    static Ptr Create(const S& state,
                      const ConstPtr& child,
                      const Trajectory<S>& traj);

  private:
    explicit Node<S>() {}
    explicit Node<S>(const S& state,
                     const ConstPtr& child,
                     const Trajectory<S>& traj) {}
  }; //\struct Node<S>

  // Implementation of static factory methods for constructing a Node.
  template<typename S>
  Node<S>::Ptr Node<S>::Create() { return Node<S>::Ptr(new Node<S>()); }

  template<typename S>
  Node<S>::Ptr Node<S>::Create(const S& state,
                               const Node<S>::ConstPtr& child,
                               const Trajectory<S>& traj) {
    return Node<S>::Ptr(new Node<S>(state, child, traj));
  }

}; //\class GraphDynamicPlanner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Recursive version of Plan() that plans outbound and return trajectories.
// High level recursive feasibility logic is here.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::
RecursivePlan(const S& start, const S& goal, double start_time, bool outbound) {
  bool done = false;

  // Searchable sets of viable and potentially unviable nodes.
  SearchableSet< Node<S>, S > viable_states;
  SearchableSet< Node<S>, S > undecided_states;

  while (!done) {
    // Sample a new point.
  }
}

} //\namespace planning
} //\namespace fastrack

#endif
