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

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/planning/planner.h>
#include <fastrack/trajectory/trajectory.h>
#include <fastrack/utils/searchable_set.h>
#include <fastrack/utils/types.h>

namespace fastrack {
namespace planning {

using dynamics::Dynamics;

template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
class GraphDynamicPlanner : public Planner<S, E, D, SD, B, SB> {
 public:
  virtual ~GraphDynamicPlanner() {}

 protected:
  explicit GraphDynamicPlanner() : Planner<S, E, D, SD, B, SB>() {}

  // Load parameters.
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  Trajectory<S> Plan(const S& start, const S& goal,
                     double start_time = 0.0) const;

  // Generate a sub-plan that connects two states and is dynamically feasible
  // (but not necessarily recursively feasible).
  virtual Trajectory<S> SubPlan(const S& start, const S& goal,
                                double start_time = 0.0) const = 0;

  // Cost functional. Defaults to time, but can be overridden.
  virtual double Cost(const Trajectory<S>& traj) const {
    return traj.Duration();
  }

  // Member variables.
  size_t num_neighbors_;
  double search_radius_;

  // Node in implicit planning graph, templated on state type.
  // NOTE! To avoid memory leaks, Nodes are constructed using a static
  // factory method that returns a shared pointer.
  struct Node {
    // Typedefs.
    typedef std::shared_ptr<Node> Ptr;
    typedef std::shared_ptr<const Node> ConstPtr;

    // Member variables.
    S state;
    double time = constants::kInfinity;
    double cost_to_come = constants::kInfinity;
    bool is_viable = false;
    Node::Ptr best_parent = nullptr;
    std::vector<Node::Ptr> children;
    std::vector<Trajectory<S>> trajs_to_children;

    // Factory methods.
    static Node::Ptr Create() { return Node::Ptr(new Node()); }
    static Node::Ptr Create(
        const S& state, double time, double cost_to_come, bool is_viable,
        const Node::Ptr& best_parent, const std::vector<Node::Ptr>& children,
        const std::vector<Trajectory<S>>& trajs_to_children) {
      return Node::Ptr(new Node(state, time, cost_to_come, is_viable,
                                best_parent, children, trajs_to_children));
    }

   private:
    explicit Node() {}
    explicit Node(const S& this_state, double this_time,
                  double this_cost_to_come, bool this_is_viable,
                  const Node::Ptr& this_best_parent,
                  const std::vector<Node::Ptr>& this_children,
                  const std::vector<Trajectory<S>>& this_trajs_to_children)
        : state(this_state),
          time(this_time),
          cost_to_come(this_cost_to_come),
          is_viable(this_is_viable),
          best_parent(this_best_parent),
          children(this_children),
          trajs_to_children(this_trajs_to_children) {}
  };  //\struct Node

  // Recursive version of Plan() that plans outbound and return trajectories.
  // High level recursive feasibility logic is here. Keep track of the
  // graph of explored states, a set of goal states, the start time,
  // whether or not this is an outbound or return trip, and whether
  // or not to extract a trajectory at the end (if not, returns an empty one.)
  Trajectory<S> RecursivePlan(SearchableSet<Node, S>& graph,
                              const SearchableSet<Node, S>& goals,
                              double start_time, bool outbound,
                              const ros::Time& initial_call_time) const;

  // Extract a trajectory from goal node to start node if one exists.
  // Returns empty trajectory if none exists.
  Trajectory<S> ExtractTrajectory(const typename Node::ConstPtr& start,
                                  const typename Node::ConstPtr& goal) const;

  // Update cost to come, time, and all traj_to_child times recursively.
  void UpdateDescendants(const typename Node::Ptr& node,
                         const typename Node::ConstPtr& start) const;

};  //\class GraphDynamicPlanner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::Plan(
    const S& start, const S& goal, double start_time) const {
  // Keep track of initial time.
  const ros::Time initial_call_time = ros::Time::now();

  // Set up start and goal nodes.
  typename Node::Ptr start_node;
  start_node->state = start;
  start_node->time = start_time;
  start_node->cost_to_come = 0.0;
  start_node->is_viable = true;

  typename Node::Ptr goal_node;
  goal_node->state = goal;
  goal_node->time = constants::kInfinity;
  goal_node->cost_to_come = constants::kInfinity;
  goal_node->is_viable = true;

  // Generate trajectory.
  SearchableSet<Node, S> graph(start_node);
  const SearchableSet<Node, S> goal_set(goal_node);
  const Trajectory<S> traj =
      RecursivePlan(graph, goal_set, start_time, true, initial_call_time);

  // Wait around if we finish early.
  const double elapsed_time = (ros::Time::now() - initial_call_time).toSec();
  if (elapsed_time < this->max_runtime_)
    ros::Duration(this->max_runtime_ - elapsed_time).sleep();

  return traj;
}

// Recursive version of Plan() that plans outbound and return trajectories.
// High level recursive feasibility logic is here.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::RecursivePlan(
    SearchableSet<Node, S>& graph, const SearchableSet<Node, S>& goals,
    double start_time, bool outbound,
    const ros::Time& initial_call_time) const {
  // Loop until we run out of time.
  while ((ros::Time::now() - initial_call_time).toSec() < this->max_runtime_) {
    // (1) Sample a new point.
    const S sample = S::Sample();

    // (2) Get k nearest neighbors.
    const std::vector<typename Node::Ptr> neighbors =
        graph.KnnSearch(sample, this->num_neighbors_);

    typename Node::Ptr sample_node = nullptr;
    for (const auto& neighbor : neighbors) {
      // Reject this neighbor if it's too close to the sample.
      if ((neighbor->state.ToVector() - sample.ToVector()).norm() <
          constants::kEpsilon)
        continue;

      // (3) Plan a sub-path from this neighbor to sampled state.
      const Trajectory<S> sub_plan =
          SubPlan(neighbor->state, sample, neighbor->time);

      if (sub_plan.Size() > 0) {
        typename Node::Ptr parent = neighbor;

        // Add to graph.
        sample_node = Node::Create();
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
    if (sample_node == nullptr) continue;

    // (4) Connect to one of the k nearest goal states if possible.
    std::vector<typename Node::Ptr> neighboring_goals =
        goals.RadiusSearch(sample, search_radius_);

    typename Node::Ptr child = nullptr;
    for (const auto& goal : neighboring_goals) {
      // Check if this is a viable node.
      if (!goal->is_viable) continue;

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
        const Trajectory<S> ignore = RecursivePlan(
            graph, graph, sample_node->time, false, initial_call_time);
    } else {
      // Reached the goal. Update goal to ensure it always has the
      // best parent.
      if (child->best_parent == nullptr ||
          child->best_parent->cost_to_come > sample_node->cost_to_come) {
        // I'm yo daddy.
        child->best_parent = sample_node;

        // Breath first search to update time / cost to come.
        // Will halt at the start node, which must be set carefully depending
        // upon whether we are going outbound (away from the start) or not
        // (toward the start).
        const auto& start_node =
            (outbound) ? graph.InitialNode() : goals.InitialNode();
        UpdateDescendants(sample_node, start_node);
      }

      // Make sure all ancestors are viable.
      // NOTE! Worst parents are not going to get updated.
      typename Node::Ptr parent = sample_node;
      while (parent != nullptr && !parent->is_viable) {
        parent->is_viable = true;
        parent = parent->best_parent;
      }

      // Extract trajectory. Always walk backward from the initial node of the
      // goal set to that of the start set.
      // Else, return a dummy trajectory since it will be ignored anyway.
      if (outbound)
        return ExtractTrajectory(graph.InitialNode(), goals.InitialNode());
      else
        return Trajectory<S>();
    }
  }

  // Ran out of time.
  ROS_ERROR("%s: Planner ran out of time.", this->name_.c_str());

  // Don't return a trajectory if not outbound.
  if (!outbound) return Trajectory<S>();

  // Return a viable loop if we found one.
  const typename Node::ConstPtr start = graph.InitialNode();
  if (start->best_parent == nullptr) {
    ROS_ERROR("%s: No viable loops available.", this->name_.c_str());
    return Trajectory<S>();
  }

  ROS_INFO("%s: Found a viable loop.", this->name_.c_str());
  return ExtractTrajectory(start, start);
}

// Extract a trajectory from goal node to start node if one exists.
// Returns empty trajectory if none exists.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::ExtractTrajectory(
    const typename Node::ConstPtr& start,
    const typename Node::ConstPtr& goal) const {
  // Accumulate trajectories in a list.
  std::list<Trajectory<S>> trajs;

  typename Node::ConstPtr node = goal;
  while (node != start || (node == start && trajs.size() == 0)) {
    const typename Node::ConstPtr parent = node->best_parent;

    if (parent == nullptr) {
      ROS_ERROR("%s: Parent was null.", this->name_.c_str());
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
        ROS_ERROR("%s: Parent/child inconsistency.", this->name_.c_str());
    }
  }

  // Concatenate into a single trajectory.
  return Trajectory<S>(trajs);
}

// Load parameters.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
bool GraphDynamicPlanner<S, E, D, SD, B, SB>::LoadParameters(
    const ros::NodeHandle& n) {
  if (!Planner<S, E, D, SD, B, SB>::LoadParameters(n)) return false;

  ros::NodeHandle nl(n);

  // Search parameters.
  int k;
  if (!nl.getParam("search_radius", this->search_radius_)) return false;
  if (!nl.getParam("num_neighbors", k)) return false;
  this->num_neighbors_ = static_cast<size_t>(k);

  return true;
}

// Update cost to come, time, and all traj_to_child times recursively.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateDescendants(
    const typename Node::Ptr& node,
    const typename Node::ConstPtr& start) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list<typename Node::Ptr> queue = {node};

  while (queue.size() > 0) {
    // Pop oldest node.
    const typename Node::Ptr current_node = queue.front();
    queue.pop_front();

    // Skip this one if it's the start node.
    if (current_node == start) continue;

    for (size_t ii = 0; ii < current_node->children.size(); ii++) {
      const typename Node::Ptr child = current_node->children[ii];

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

        child->time =
            current_node->time + current_node->trajs_to_children[ii].Duration();
        child->cost_to_come = current_node->cost_to_come +
                              Cost(current_node->trajs_to_children[ii]);
      }
    }
  }
}

}  // namespace planning
}  // namespace fastrack

#endif
