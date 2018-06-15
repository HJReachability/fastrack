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

  // Load parameters.
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  Trajectory<S> Plan(const S& start, const S& goal, double start_time=0.0) const;

  // Recursive version of Plan() that plans outbound and return trajectories.
  // High level recursive feasibility logic is here. Keep track of the
  // graph of explored states, a set of goal states, the start time,
  // whether or not this is an outbound or return trip, and whether
  // or not to extract a trajectory at the end (if not, returns an empty one.)
  Trajectory<S> RecursivePlan(SearchableSet< Node<S>, S >& graph,
                              const SearchableSet< Node<S>, S >& goals,
                              double start_time,
                              bool outbound,
                              bool extract_traj,
                              const ros::Time& initial_call_time) const;

  // Generate a sub-plan that connects two states and is dynamically feasible
  // (but not necessarily recursively feasible).
  virtual Trajectory<S> SubPlan(
    const S& start, const S& goal, double start_time=0.0) const = 0;

  // Cost functional. Defaults to time, but can be overridden.
  virtual double Cost(const Trajectory<S>& traj) const { return traj.Duration(); }

  // Extract a trajectory from goal node to start node if one exists.
  // Returns empty trajectory if none exists.
  Trajectory<S> ExtractTrajectory(const Node<S>::ConstPtr& start,
                                  const Node<S>::ConstPtr& goal) const;

  // Update cost to come, time, and all traj_to_child times recursively.
  void UpdateChildren(const Node<S>::Ptr& node) const;

  // Member variables.
  size_t num_neighbors_;
  double search_radius_;

  // Node in implicit planning graph, templated on state type.
  // NOTE! To avoid memory leaks, Nodes are constructed using a static
  // factory method that returns a shared pointer.
  template<typename S>
  struct Node<S> {
    // Member variables.
    S state;
    double time = constants::INFINITY;
    double cost_to_come = constants::INFINITY;
    bool is_viable = false;
    Node<S>::ConstPtr best_parent = nullptr;
    std::vector< Node<S>::Ptr > children;
    std::vector< Trajectory<S> > trajs_to_children;

    // Typedefs.
    typedef std::shared_ptr< Node<S> > Ptr;
    typedef std::shared_ptr< const Node<S> > ConstPtr;

    // Factory methods.
    static Node<S>::Ptr Create();
    static Node<S>::Ptr Create(
      const S& state,
      double time,
      double cost_to_come,
      bool is_viable,
      const Node<S>::Ptr& best_parent,
      const std::vector< Node<S>::Ptr >& children,
      const std::vector< Trajectory<S> >& trajs_to_children);

  private:
    explicit Node<S>() {}
    explicit Node<S>(const S& state,
                     double time,
                     double cost_to_come,
                     bool is_viable,
                     const Node<S>::Ptr& best_parent,
                     const std::vector< Node<S>::Ptr >& children,
                     const std::vector< Trajectory<S> >& trajs_to_children)
    : this->state(state),
      this->time(time),
      this->cost_to_come(cost_to_come),
      this->is_viable(is_viable),
      this->best_parent(best_parent),
      this->children(children),
      this->trajs_to_children(trajs_to_children) {}
  }; //\struct Node<S>

  // Implementation of static factory methods for constructing a Node.
  template<typename S>
  Node<S>::Ptr Node<S>::Create() { return Node<S>::Ptr(new Node<S>()); }

  template<typename S>
  Node<S>::Ptr Node<S>::
  Create(const S& state,
         bool is_viable,
         double time,
         double cost_to_come,
         const Node<S>::Ptr& best_parent,
         const std::vector< Node<S>::Ptr >& children,
         const std::vector< Trajectory<S> >& trajs_to_children) {
    return Node<S>::Ptr(new Node<S>(state,
                                    time,
                                    cost_to_come,
                                    is_viable,
                                    best_parent,
                                    children,
                                    trajs_to_children));
  }

}; //\class GraphDynamicPlanner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::
Plan(const S& start, const S& goal, double start_time=0.0) const {
  // Keep track of initial time.
  const ros::Time initial_call_time = ros::Time::now();

  // Set up start and goal nodes.
  Node<S>::Ptr start_node;
  start_node->state = start;
  start_node->time = start_time;
  start_node->cost_to_come = 0.0;
  start_node->is_viable = true;

  Node<S>::Ptr goal_node;
  goal_node->state = goal;
  goal_node->time = constants::INFINITY;
  goal_node->cost_to_come = constants::INFINITY;
  goal_node->is_viable = true;

  // Generate trajectory.
  const Trajectory<S> traj =
    RecursivePlan(SearchableSet< Node<S>, S >(start_node),
                  SearchableSet< Node<S>, S >(goal_node),
                  start_time, true, true, initial_call_time);

  // Wait around if we finish early.
  const double elapsed_time = (ros::Time::now() - initial_call_time).toSec();
  if (elapsed_time < max_runtime_)
    ros::Duration(max_runtime_ - elapsed_time).sleep();

  return traj;
}


// Recursive version of Plan() that plans outbound and return trajectories.
// High level recursive feasibility logic is here.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::
RecursivePlan(SearchableSet< Node<S>, S >& graph,
              const SearchableSet< Node<S>, S >& goals,
              double start_time,
              bool outbound,
              bool extract_traj,
              const ros::Time& initial_call_time) const {
  // Loop until we run out of time.
  while ((ros::Time::now() - initial_call_time).toSec() < max_runtime_) {
    // (1) Sample a new point.
    const S sample = S::Sample(lower, upper);

    // (2) Get k nearest neighbors.
    const std::vector< Node<S>::Ptr > neighbors =
      graph.KnnSearch(sample, num_neighbors_);

    Node<S>::Ptr parent = nullptr;
    for (const auto& neighbor : neighbors) {
      // Reject this neighbor if it's too close to the sample.
      if ((neighbor->state.ToVector() - sample.ToVector()).norm() <
          constants::EPSILON)
        continue;

      // (3) Plan a sub-path from this neighbor to sampled state.
      const Trajectory<S> sub_plan =
        SubPlan(neighbor->state, sample, neighbor->time);

      if (sub_plan.Size() > 0) {
        parent = neighhor;

        // Add to graph.
        Node<S>::Ptr sample_node = Node<S>::Create();
        sample_node->state = sample;
        sample_node->time = parent->time + sub_plan.Duration();
        sample_node->cost_to_come = parent->cost_to_come + Cost(sub_plan);
        sample_node->is_viable = false;
        sample_node->best_parent = parent;
        sample_node->children = {};
        sample_node->trajs_to_children = {};

        // Update parent.
        parent->children.push_back(sample_node);
        parent->trajs_to_children.push_back(sub_plan);

        break;
      }
    }

    // Sample a new point if there was no good way to get here.
    if (parent == nullptr)
      continue;

    // (4) Connect to one of the k nearest goal states if possible.
    std::vector< Node<S>::Ptr > neighboring_goals =
      goals.RadiusSearch(sample, search_radius_);

    Node<S>::Ptr child = nullptr;
    for (const auto& goal : neighboring_goals) {
      // Check if this is a viable node.
      if (!goal->is_viable)
        continue;

      // Try to connect.
      const Trajectory<S> sub_plan =
        SubPlan(sample, goal->state, sample_node->time);

      // Upon success, set child to point to goal and update sample node to
      // include child node and corresponding trajectory.
      if (sub_plan.Size() > 0) {
        child = goal;

        // Update sample node to point to child.
        sample_node->children.push_back(child);
        sample_node->trajs_to_children.push_back(sub_plan);

        break;
      }
    }

    if (child == nullptr) {
      // (5) If outbound, make a recursive call.
      if (outbound)
        const Trajectory<S> ignore =
          RecursivePlan(graph, graph, sample_node->time,
                        false, false, initial_call_time);
    } else {
      // Reached the goal. Update goal to ensure it always has the
      // best parent.
      if (child->best_parent == nullptr ||
          child->best_parent->cost_to_come > sample_node->cost_to_come) {
        // I'm yo daddy.
        child->best_parent = sample_node;

        // Breath first search to update time / cost to come.
        UpdateChildren(sample_node);
      }

      // Make sure all ancestors are viable.
      // NOTE! Worst parents are not going to get updated.
      Node<S>::Ptr parent = sample_node;
      while (parent != nullptr && !parent->is_viable) {
        parent->is_viable = true;
        parent = parent->best_parent;
      }

      // Extract trajectory. Always walk backward from the initial node of the
      // goal set to that of the start set.
      // Else, return a dummy trajectory since it will be ignored anyway.
      if (extract_traj)
        return ExtractTrajectory(graph.InitialNode(), goals.InitialNode())
      else
        return Trajectory<S>();
    }
  }

  // Ran out of time.
  ROS_ERROR("%s: Planner ran out of time.", name_.c_str());

  // Return a viable loop if we found one.
  const Node<S>::ConstPtr start = graph.InitialNode();
  if (start->best_parent == nullptr) {
    ROS_ERROR("%s: No viable loops available.", name_.c_str());
    return Trajectory<S>();
  }

  ROS_INFO("%s: Found a viable loop.", name_.c_str());
  return ExtractTrajectory(start, start);
}

// Extract a trajectory from goal node to start node if one exists.
// Returns empty trajectory if none exists.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::
ExtractTrajectory(const Node<S>::ConstPtr& start,
                  const Node<S>::ConstPtr& goal) const {
  // Accumulate trajectories in a list.
  std::list< Trajectory<S> > trajs;

  Node<S>::Ptr node = goal;
  while (node != start || (node == start && trajs.size() == 0)) {
    const Node<S>::Ptr parent = node->best_parent;

    if (parent == nullptr) {
      ROS_ERROR("%s: Parent was null.", name_.c_str());
      break;
    }

    // Find node as child of parent.
    // NOTE! Could avoid this by replacing parallel std::vectors
    //       an std::unordered_map.
    for (size_t ii = 0; ii < parent->children.size(); ii++) {
      if (parent->children[ii] == node) {
        trajs.push_front(parent->trajs_to_children[ii]);
        break;
      }

      if (ii == parent->children.size() - 1)
        ROS_ERROR("%s: Parent/child inconsistency.", name_.c_str());
    }
  }

  // Concatenate into a single trajectory.
  return Trajectory<S>(trajs);
}

// Load parameters.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
bool GraphDynamicPlanner<S, E, D, SD, B, SB>::
LoadParameters(const ros::NodeHandle& n) {
  if (!Planner<S, E, D, SD, B, SB>::LoadParameters(n))
    return false;

  ros::NodeHandle nl(n);

  // Search parameters.
  int k;
  if (!nl.getParam("planner/search_radius", search_radius_)) return false;
  if (!nl.getParam("planner/num_neighbors", k)) return false;
  num_neighbors_ = static_cast<size_t>(k);

  return true;
}

// Update cost to come, time, and all traj_to_child times recursively.
template<typename S, typename E,
         typename D, typename SD, typename B, typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::
UpdateChildren(const Node<S>::Ptr& node) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list< Node<S>::Ptr > queue = { node };

  while (queue.size() > 0) {
    // Pop oldest node.
    const Node<S>::Ptr current_node = queue.front();
    queue.pop_front();

    for (size_t ii = 0; ii < current_node->children.size(); ii++) {
      const Node<S>::Ptr child = current_node->children[ii];

      // Push child onto the queue.
      queue.push_back(child);

      // Update trajectory to child.
      // NOTE! This could be removed since trajectory timing is adjusted
      //       again upon concatenation and extraction.
      current_node->trajs_to_children[ii].ResetFirstTime(current_node->time);

      // Maybe update child's best parent to be the current node.
      // If so, also update time and cost to come.
      if (child->best_parent != nullptr ||
          child->best_parent->cost_to_come > current_node->cost_to_come) {
        child->best_parent = current_node;

        child->time = current_node->time +
          current_node->trajs_to_children[ii].Duration();
        child->cost_to_come = current_node->cost_to_come +
          Cost(current_node->trajs_to_children[ii]);
      }
    }
  }
}


} //\namespace planning
} //\namespace fastrack

#endif
