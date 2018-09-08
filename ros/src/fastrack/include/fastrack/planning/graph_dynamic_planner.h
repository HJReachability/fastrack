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
 *          Jaime F. Fisac            ( jfisac@eecs.berkeley.edu )
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

#include <unordered_map>

namespace fastrack {
namespace planning {

namespace {
// Return false if the given trajectory does not start/end at the specified
// states.
template <typename S>
static bool CheckTrajectoryEndpoints(const Trajectory<S>& traj, const S& start,
                                     const S& end) {
  if (!traj.FirstState().ToVector().isApprox(start.ToVector(),
                                             constants::kEpsilon)) {
    ROS_ERROR_STREAM(
        "Planner returned a trajectory with the wrong start state: "
        << traj.FirstState().ToVector().transpose() << " vs. "
        << start.ToVector().transpose());
    return false;
  }

  if (!traj.LastState().ToVector().isApprox(end.ToVector(),
                                            constants::kEpsilon)) {
    ROS_ERROR_STREAM("Planner returned a trajectory with the wrong end state: "
                     << traj.LastState().ToVector().transpose() << " vs. "
                     << end.ToVector().transpose());
    return false;
  }

  return true;
}

}  //\namespace

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
    double cost_to_home = constants::kInfinity;
    double cost_to_goal = constants::kInfinity;
    bool is_viable = false;
    Node::Ptr best_parent = nullptr;
    Node::Ptr best_home_child = nullptr;
    Node::Ptr best_goal_child = nullptr;
    std::vector<Node::Ptr> parents;
    std::unordered_map<Node::Ptr, Trajectory<S>> trajs_to_children;

    // Factory methods.
    static Node::Ptr Create() { return Node::Ptr(new Node()); }

    // Operators for equality checking.
    bool operator==(const Node& other) const {
      constexpr double kSmallNumber = 1e-8;
      return state.ToVector().isApprox(other.state.ToVector(), kSmallNumber);
    }

    bool operator!=(const Node& other) const { return !(*this == other); }

   private:
    Node() {}
  };  //\struct Node

  // Recursive version of Plan() that plans outbound and return trajectories.
  // High level recursive feasibility logic is here. Keep track of the
  // graph of explored states, a set of goal states, the start time,
  // whether or not this is an outbound or return trip, and whether
  // or not to extract a trajectory at the end (if not, returns an empty one.)
  Trajectory<S> RecursivePlan(const typename Node::Ptr& start_node,
                              const SearchableSet<Node, S>& goals,
                              double start_time, bool outbound,
                              const ros::Time& initial_call_time) const;

  // Extract a trajectory from start node to goal node if one exists.
  // Returns empty trajectory if none exists. If goal node is the home
  // node, then will also include trajectories from the home node back
  // to the start node to form a loop, i.e. start->home->start.
  // Trajectory will begin at the specified time.
  // NOTE: this function updates 'traj_nodes_' and 'traj_node_times_'.
  Trajectory<S> ExtractTrajectory(const typename Node::Ptr& start,
                                  const typename Node::Ptr& goal,
                                  double start_time) const;

  // Update cost to come, best parent, time, and all traj_to_child times
  // recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateDescendants(const typename Node::Ptr& node) const;

  // Update cost to home and best home child recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateAncestorsCostToHome(const typename Node::Ptr& node) const;

  // Update cost to goal and best goal child recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateAncestorsCostToGoal(const typename Node::Ptr& node) const;

  // Number of neighbors and radius to use for nearest neighbor searches.
  size_t num_neighbors_;
  double search_radius_;

  // Backward-reachable set of the "home" state.
  // This is the initial state of the planner which we assume is viable.
  mutable std::unique_ptr<SearchableSet<Node, S>> home_set_;

  // Parallel lists of nodes and times.
  // These correspond to the most recently output trajectory and are used
  // for quickly identifying query start states on the graph.
  // NOTE: these times are absolute ROS times, not relative to 0.0.
  mutable std::vector<typename Node::Ptr> traj_nodes_;
  mutable std::vector<double> traj_node_times_;
};  //\class GraphDynamicPlanner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.o
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::Plan(
    const S& start, const S& goal, double start_time) const {
  // Keep track of initial time.
  const ros::Time initial_call_time = ros::Time::now();

  // Set up goal node.
  typename Node::Ptr goal_node = Node::Create();
  goal_node->state = goal;
  goal_node->time = constants::kInfinity;
  goal_node->cost_to_come = constants::kInfinity;
  goal_node->cost_to_home = constants::kInfinity;
  goal_node->cost_to_goal = 0.0;
  goal_node->is_viable = true;

  // Set a start node as null. This will end up either being:
  // (1) the home node, if we don't have one yet, OR
  // (2) the node in traj_nodes immediately after the start_time.
  typename Node::Ptr start_node;

  // If we have a start node that is not the home node, we will also have
  // a trajectory from the previous node to the start node that will have to
  // be concatenated to the trajectory extracted from the graph.
  std::unique_ptr<Trajectory<S>> traj_to_start_node;

  // Check if we have a home_set. If not, create one from this start state.
  // NOTE: assign home node a time of 0.0, since it will persist over multiple
  // planning invocations.
  // NOTE: if we don't yet have a home set, we also will not have a previous
  // node trajectory.
  if (!home_set_) {
    typename Node::Ptr home_node = Node::Create();
    home_node->state = start;
    home_node->time = start_time;
    home_node->cost_to_come = 0.0;
    home_node->cost_to_home = 0.0;
    home_node->cost_to_goal = constants::kInfinity;
    home_node->is_viable = true;
    home_node->time = 0.0;

    home_set_.reset(new SearchableSet<Node, S>(home_node));
    start_node = home_node;
  } else if (!traj_nodes_.empty()) {
    // We already have a home set, however we may not have traj_nodes yet
    // because the first few planner invocations may fail.

    // Find which two nodes we are between.
    // NOTE: we will assume that the start time is strictly within the
    // time interval specified by the 'traj_node_times_'.
    const auto& iter = std::lower_bound(traj_node_times_.begin(),
                                        traj_node_times_.end(), start_time);
    if (iter == traj_node_times_.end()) {
      throw std::runtime_error("Invalid start time: too late.");
    }

    if (iter == traj_node_times_.begin()) {
      throw std::runtime_error("Invalid start time: too early.");
    }

    const size_t hi = std::distance(traj_node_times_.begin(), iter);
    const size_t lo = hi - 1;

    const auto& next_node = traj_nodes_[hi];
    const auto& previous_node = traj_nodes_[lo];
    const double previous_node_time = traj_node_times_[lo];

    // Go ahead and extract the partial trajectory from previous to next node.
    // This lives in the previous node.
    const auto& traj_iter = previous_node->trajs_to_children.find(next_node);
    if (traj_iter == previous_node->trajs_to_children.end()) {
      ROS_ERROR("%s: Inconsistency between parent and child.",
                this->name_.c_str());
    }

    // NOTE: we don't have to truncate this trajectory because the interpolator
    // in PlannerManager will take care of it for us!
    // NOTE: ExtractTrajectory() will convert trajectories' relative times to
    // absolute ROS times based on start_time.
    traj_to_start_node.reset(new Trajectory<S>(traj_iter->second));
    traj_to_start_node->ResetFirstTime(previous_node_time);

    // Set start_node to next_node.
    start_node = next_node;
  }

  // Generate trajectory. This trajectory will originate from the start node and
  // either terminate within the goal set OR at the start_node and pass through
  // the home node.
  const SearchableSet<Node, S> goal_set(goal_node);
  const double plan_start_time =
      (traj_to_start_node) ? traj_to_start_node->LastTime() : start_time;
  const Trajectory<S> traj = RecursivePlan(
      start_node, goal_set, plan_start_time, true, initial_call_time);

  // Wait around if we finish early.
  const double elapsed_time = (ros::Time::now() - initial_call_time).toSec();
  if (elapsed_time < this->max_runtime_)
    ros::Duration(this->max_runtime_ - elapsed_time).sleep();

  // If we have a trajectory from the start state to the start node, concatenate
  // with the planned trajectory.
  if (traj_to_start_node) {
    return Trajectory<S>(std::list<Trajectory<S>>({*traj_to_start_node, traj}));
  }

  return traj;
}

// Recursive version of Plan() that plans outbound and return trajectories.
// High level recursive feasibility logic is here.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::RecursivePlan(
    const typename Node::Ptr& start_node, const SearchableSet<Node, S>& goals,
    double start_time, bool outbound,
    const ros::Time& initial_call_time) const {
  // Loop until we run out of time.
  while ((ros::Time::now() - initial_call_time).toSec() < this->max_runtime_) {
    // (1) Sample a new point.
    const S sample = S::Sample();
    //    std::cout << "sample: " << sample.ToVector().transpose() << std::endl;

    // Reject this sample if it's not in known free space.
    if (!this->env_.AreValid(sample.OccupiedPositions(), this->bound_))
      continue;

    // Check the home set for nearest neighbors and connect.
    std::vector<typename Node::Ptr> home_set_neighbors =
        home_set_->KnnSearch(sample, num_neighbors_);

    typename Node::Ptr parent = nullptr;
    typename Node::Ptr sample_node = nullptr;
    for (const auto& neighboring_parent : home_set_neighbors) {
      // (2) Plan a sub-path from the start to the sampled state.
      const Trajectory<S> sub_plan =
          SubPlan(neighboring_parent->state, sample, start_node->time);
      if (sub_plan.Size() == 0) continue;

      // If somehow the planner returned a plan that does not terminate at the
      // desired goal, or begin at the desired start, then discard.
      if (!CheckTrajectoryEndpoints(sub_plan, neighboring_parent->state,
                                    sample))
        continue;

      // std::cout << "Found a subplan of length: " << sub_plan.Size() <<
      // std::endl;

      // Add to graph.
      sample_node = Node::Create();
      sample_node->state = sample;
      sample_node->time = neighboring_parent->time + sub_plan.Duration();
      sample_node->cost_to_come =
          neighboring_parent->cost_to_come + Cost(sub_plan);
      sample_node->cost_to_home = constants::kInfinity;
      sample_node->is_viable = false;
      sample_node->best_parent = neighboring_parent;
      sample_node->best_home_child = nullptr;
      sample_node->parents = {neighboring_parent};
      sample_node->trajs_to_children = {};

      // Update parent.
      neighboring_parent->trajs_to_children.emplace(sample_node, sub_plan);

      // Set parent to neighboring parent and get of here.
      parent = neighboring_parent;
      break;
    }

    // Skip this sample if we didn't find a parent.
    if (!parent) continue;

    // Sample node had better not be null.
    if (!sample_node) {
      throw std::runtime_error("Sample node was null.");
    }

    // (3) Connect to one of the k nearest goal states if possible.
    std::vector<typename Node::Ptr> neighboring_goals =
        goals.KnnSearch(sample, num_neighbors_);

    typename Node::Ptr child = nullptr;
    for (const auto& goal : neighboring_goals) {
      // Check if this is a viable node.
      // NOTE: inbound recursive calls may encounter nodes which are goals (i.e.
      // in the home set) but NOT viable yet.
      if (!goal->is_viable) {
        if (outbound) {
          ROS_ERROR_THROTTLE(1.0, "%s: Goal was not viable on outbound call.",
                             this->name_.c_str());
        }

        continue;
      }

      // Check if goal is in known free space.
      if (!this->env_.AreValid(goal->state.OccupiedPositions(), this->bound_)) {
        ROS_INFO_THROTTLE(1.0, "%s: Goal was not in known free space.",
                          this->name_.c_str());
        continue;
      }

      // Try to connect.
      const Trajectory<S> sub_plan =
          SubPlan(sample, goal->state, sample_node->time);
      if (sub_plan.Size() == 0) continue;

      // Upon success, set child to point to goal and update sample node to
      // include child node and corresponding trajectory.
      // If somehow the planner returned a plan that does not terminate at the
      // desired goal, or begin at the desired start, then discard.
      if (!CheckTrajectoryEndpoints(sub_plan, sample_node->state, goal->state))
        continue;

      // Set child to goal, since we have a valid trajectory to get there.
      child = goal;

      // Add ourselves as a parent of the goal.
      goal->parents.push_back(sample_node);

      // Update sample node to point to child.
      sample_node->trajs_to_children.emplace(child, sub_plan);
      sample_node->is_viable = true;
      break;
    }

    if (child == nullptr) {
      // (5) If outbound, make a recursive call. We can ignore the returned
      // trajectory since we'll generate one later once we're all done.
      if (outbound) {
        const Trajectory<S> ignore = RecursivePlan(
            sample_node, *home_set_, start_time, false, initial_call_time);
      }
    } else {
      // Reached the goal. This means that 'sample_node' now has exactly one
      // viable child. We need to update all sample node's descendants to
      // reflect the potential change in best parent of sample_node's child.
      // Also, sample node is now viable, so mark all ancestors as viable.
      UpdateDescendants(sample_node);

      if (outbound) {
        // Update sample node's cost to goal.
        sample_node->cost_to_goal =
            sample_node->cost_to_come +
            Cost(sample_node->trajs_to_children.at(child));

        // Propagate backward to all ancestors.
        UpdateAncestorsCostToGoal(sample_node);
      } else {
        // Update sample node's cost to home.
        sample_node->cost_to_home =
            sample_node->cost_to_come +
            Cost(sample_node->trajs_to_children.at(child));

        // Propagate backward to all ancestors.
        UpdateAncestorsCostToHome(sample_node);
      }

      // Extract trajectory. Always walk backward from the initial node of the
      // goal set to the start node. Be sure to set the correct start time.
      // NOTE: this will automatically set traj_nodes and traj_node_times.
      // Else, return a dummy trajectory since it will be ignored anyway.
      if (outbound) {
        return ExtractTrajectory(start_node, goals.InitialNode(), start_time);
      } else {
        return Trajectory<S>();
      }
    }
  }

  // Ran out of time.
  ROS_ERROR("%s: Planner ran out of time.", this->name_.c_str());

  // Don't return a trajectory if not outbound.
  if (!outbound) return Trajectory<S>();

  // Return a viable loop if we found one.
  const typename Node::Ptr home_node = home_set_->InitialNode();
  if (home_node->parents.empty()) {
    ROS_ERROR("%s: No viable loops available.", this->name_.c_str());
    return Trajectory<S>();
  }

  ROS_INFO("%s: Found a viable loop.", this->name_.c_str());
  return ExtractTrajectory(start_node, home_node, start_time);
}

// Extract a trajectory from start node to goal node if one exists.
// Returns empty trajectory if none exists. If goal node is the home
// node, then will also include trajectories from the home node back
// to the start node to form a loop, i.e. start->home->start.
// Trajectory will begin at the specified time.
// NOTE: this function updates 'traj_nodes_' and 'traj_node_times_'.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::ExtractTrajectory(
    const typename Node::Ptr& start, const typename Node::Ptr& goal,
    double start_time) const {
  // If we have traj_nodes and traj_node_times, interpolate at the given time
  // and reset to include only the node/time preceding the start_time.
  // The node/time succeeding the start_time will be added later in this method.
  if (!traj_nodes_.empty()) {
    const auto iter = std::lower_bound(traj_node_times_.begin(),
                                       traj_node_times_.end(), start_time);
    if (iter == traj_node_times_.end()) {
      throw std::runtime_error("Invalid start time: too late.");
    }

    if (iter == traj_node_times_.begin()) {
      throw std::runtime_error("Invalid start time: too early.");
    }

    const size_t hi = std::distance(traj_node_times_.begin(), iter);
    const size_t lo = hi - 1;

    const auto& lo_node = traj_nodes_[lo];
    const double lo_time = traj_node_times_[lo];

    traj_nodes_ = std::vector<typename Node::Ptr>({lo_node});
    traj_node_times_ = std::vector<double>({lo_time});
  }

  // Accumulate trajectories and nodes in a list. Use std::list because we
  // will be populating from the back to the front.
  std::list<Trajectory<S>> trajs;
  std::list<typename Node::Ptr> nodes;

  std::cout << "Extracting trajectory from "
            << start->state.ToVector().transpose() << " to "
            << goal->state.ToVector().transpose() << std::endl;

  // If the goal is the home node, start off by walking backward from the
  // start to the home node. Note that if the start is also the goal node,
  // then this loop will do nothing and that case will be handled in the next
  // loop below.
  const auto& home_node = home_set_->InitialNode();
  if (*goal == *home_node) {
    std::cout << "Prepopulating trajectory from home to start." << std::endl;

    auto node = start;
    while (*node != *home_node) {
      const auto& parent = node->best_parent;

      if (parent == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "%s: Parent was null.", this->name_.c_str());
        break;
      }

      // Find node as child of parent.
      const auto& iter = parent->trajs_to_children.find(node);
      if (iter == parent->trajs_to_children.end()) {
        ROS_ERROR_THROTTLE(1.0, "%s: Parent/child inconsistency.",
                           this->name_.c_str());
      }

      // Add to traj and node lists. Skip if last one to avoid duplication
      // in next while loop.
      if (*parent != *home_node) {
        trajs.push_front(iter->second);
        nodes.push_front(node);
      }

      // Update node to be its parent.
      node = parent;
    }
  }

  // Now walk from the goal backward toward the start.
  auto node = goal;
  while (*node != *start || trajs.empty()) {
    const auto& parent = node->best_parent;

    if (parent == nullptr) {
      ROS_ERROR_THROTTLE(1.0, "%s: Parent was null.", this->name_.c_str());
      break;
    }

    //    std::cout << "Parent is: " << parent->state.ToVector().transpose() <<
    //    std::endl;

    // Find node as child of parent.
    const auto& iter = parent->trajs_to_children.find(node);
    if (iter == parent->trajs_to_children.end()) {
      ROS_ERROR_THROTTLE(1.0, "%s: Parent/child inconsistency.",
                         this->name_.c_str());
    }

    // Add to node list.
    nodes.push_front(node);
    trajs.push_front(iter->second);

    // Update node to be its parent.
    node = parent;
    //    std::cout << "Trajs has length: " << trajs.size() << std::endl;
  }

  // Add the start node to the list of nodes.
  nodes.push_front(start);

  std::cout << "Nodes: " << std::endl;
  for (const auto& n : nodes)
    std::cout << n->state.ToVector().transpose() << std::endl;

  // Add all nodes and corresponding times to traj_nodes and traj_node_times.
  double node_time = start_time;
  auto trajs_iter = trajs.begin();
  auto nodes_iter = nodes.begin();
  while (trajs_iter != trajs.end() && nodes_iter != nodes.end()) {
    traj_nodes_.push_back(*nodes_iter);
    traj_node_times_.push_back(node_time);
    node_time += trajs_iter->Duration();

    trajs_iter++;
    nodes_iter++;
  }

  if (nodes_iter == nodes.end()) {
    throw std::runtime_error("Expecting one more node.");
  }

  traj_nodes_.push_back(*nodes_iter);
  nodes_iter++;

  if (nodes_iter != nodes.end())
    throw std::runtime_error("Incorrect number of nodes.");

  // Concatenate into a single trajectory and set initial time.
  Trajectory<S> traj(trajs);
  traj.ResetFirstTime(start_time);
  return traj;
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
  if (!nl.getParam("search_radius", search_radius_)) return false;
  if (!nl.getParam("num_neighbors", k)) return false;
  num_neighbors_ = static_cast<size_t>(k);

  return true;
}

// Update cost to come, best parent, time, and all traj_to_child times
// recursively.
// NOTE: this will never get into an infinite loop because eventually
// every node will know its best option and reject further updates.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateDescendants(
    const typename Node::Ptr& node) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list<typename Node::Ptr> queue = {node};

  // Extract home node reference.
  const auto& home_node = home_set_->InitialNode();

  while (queue.size() > 0) {
    // Pop oldest node.
    const typename Node::Ptr current_node = queue.front();
    queue.pop_front();

    // Loop over all children and add to queue if this node is their
    // NEW best parent.
    for (auto& child_traj_pair : current_node->trajs_to_children) {
      auto& child = child_traj_pair.first;
      auto& traj_to_child = child_traj_pair.second;

      // Update trajectory to child.
      traj_to_child.ResetFirstTime(current_node->time);

      // Maybe update child's best parent to be the current node.
      // If so, also update time and cost to come.
      // NOTE: if child's cost to come is infinite, it is DEFINTELY a goal.
      // Everybody else will already have a best parent, except home.
      bool new_best_parent = std::isinf(child->cost_to_come);
      const double our_cost_to_child = Cost(traj_to_child);

      if (!new_best_parent) {
        if (child->cost_to_come >
            current_node->cost_to_come + our_cost_to_child)
          new_best_parent = true;
      }

      if (new_best_parent) {
        child->best_parent = current_node;
        child->time = current_node->time + traj_to_child.Duration();
        child->cost_to_come = current_node->cost_to_come + our_cost_to_child;

        // Push child onto the queue.
        queue.push_back(child);
      }
    }
  }
}

// Update cost to home and best home child recursively.
// NOTE: this will never get into an infinite loop because eventually
// every node will know its best option and reject further updates.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateAncestorsCostToHome(
    const typename Node::Ptr& node) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list<typename Node::Ptr> queue = {node};

  // Extract home node reference.
  const auto& home_node = home_set_->InitialNode();

  while (queue.size() > 0) {
    // Pop oldest node.
    const typename Node::Ptr current_node = queue.front();
    queue.pop_front();

    // Loop over all parents and add to queue if this node is their
    // NEW best home child.
    for (auto& parent : current_node->parents) {
      // Parent is DEFINITELY viable.
      parent->is_viable = true;

      // Extract trajectory from parent to this node.
      auto& traj_from_parent = parent->trajs_to_children.at(current_node);

      // Maybe update parent's best home child to be the current node.
      // If so, also update its viability and cost to home.
      // NOTE: if parent's cost to home is infinite, we are definitely its best
      // (and only) option as best home child.
      bool new_best_home_child = std::isinf(parent->cost_to_home);
      const double our_cost_from_parent = Cost(traj_from_parent);

      if (!new_best_home_child) {
        if (parent->cost_to_home >
            current_node->cost_to_home + our_cost_from_parent)
          new_best_home_child = true;
      }

      if (new_best_home_child) {
        parent->best_home_child = current_node;
        parent->cost_to_home =
            current_node->cost_to_home + our_cost_from_parent;

        // Push child onto the queue.
        queue.push_back(parent);
      }
    }
  }
}

// Update cost to goal and best goal child recursively.
// NOTE: this will never get into an infinite loop because eventually
// every node will know its best option and reject further updates.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateAncestorsCostToGoal(
    const typename Node::Ptr& node) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list<typename Node::Ptr> queue = {node};

  while (queue.size() > 0) {
    // Pop oldest node.
    const typename Node::Ptr current_node = queue.front();
    queue.pop_front();

    // Loop over all parents and add to queue if this node is their
    // NEW best goal child.
    for (auto& parent : current_node->parents) {
      // Parent is DEFINITELY viable.
      parent->is_viable = true;

      // Extract trajectory from parent to this node.
      auto& traj_from_parent = parent->trajs_to_children.at(current_node);

      // Maybe update parent's best goal child to be the current node.
      // If so, also update its viability and cost to goal.
      // NOTE: if parent's cost to goal is infinite, we are definitely its best
      // (and only) option as best goal child.
      bool new_best_goal_child = std::isinf(parent->cost_to_goal);
      const double our_cost_from_parent = Cost(traj_from_parent);

      if (!new_best_goal_child) {
        if (parent->cost_to_goal >
            current_node->cost_to_goal + our_cost_from_parent)
          new_best_goal_child = true;
      }

      if (new_best_goal_child) {
        parent->best_goal_child = current_node;
        parent->cost_to_goal =
            current_node->cost_to_goal + our_cost_from_parent;

        // Push child onto the queue.
        queue.push_back(parent);
      }
    }
  }
}

}  // namespace planning
}  // namespace fastrack

#endif
