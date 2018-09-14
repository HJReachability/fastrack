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
#include <unordered_set>

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

// Colormap class. Keeps track of earliest and latest time stamps and provides
// a functor for mapping time to color.
class Colormap {
 public:
  ~Colormap() {}
  Colormap() : t_min_(0.0), t_max_(0.0) {}

  // Map a time to a ROS ColorRGBA.
  std_msgs::ColorRGBA operator()(double t) const {
    std_msgs::ColorRGBA color;
    color.a = 1.0;

    if (t < t_min_ || t > t_max_) {
      ROS_ERROR("Colormap: time is out of bounds.");
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
    } else {
      color.r = 0.5;
      color.g = (t - t_min_) / std::max(constants::kEpsilon, (t_max_ - t_min_));
      color.b = 1.0 - color.g;
    }

    return color;
  }

  // Update t_min and t_max.
  void UpdateTimes(double t) {
    t_min_ = std::min(t_min_, t);
    t_max_ = std::max(t_max_, t);
  }

 private:
  // Min/max timestamps.
  double t_min_, t_max_;
};  //\class Colormap

}  //\namespace

using dynamics::Dynamics;

template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
class GraphDynamicPlanner : public Planner<S, E, D, SD, B, SB> {
 public:
  virtual ~GraphDynamicPlanner() {}

 protected:
  explicit GraphDynamicPlanner() : Planner<S, E, D, SD, B, SB>(), rng_(rd_()) {}

  // Load parameters.
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  Trajectory<S> Plan(const S& start, const S& goal,
                     double start_time = 0.0) const;

  // Generate a sub-plan that connects two states and is dynamically feasible
  // (but not necessarily recursively feasible).
  virtual Trajectory<S> SubPlan(const S& start, const S& goal,
                                double start_time = 0.0) const = 0;

  // Visualize the graph.
  void Visualize() const;

  // Cost functional. Defaults to time, but can be overridden.
  virtual double Cost(const Trajectory<S>& traj) const {
    return traj.Duration();
  }

  // Heuristic function. Defaults to distance between state and goal.
  virtual double Heuristic(const S& state) const {
    if (!goal_node_) {
      ROS_ERROR("%s: Goal node was null.", this->name_.c_str());
      return constants::kInfinity;
    }

    return (state.ToVector() - goal_node_->state.ToVector()).norm();
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
    bool is_visited = false;
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
  Trajectory<S> RecursivePlan(double initial_call_time, bool outbound) const;

  // Extract a trajectory including the given start time, which either
  // loops back home or goes to the goal (if such a trajectory exists).
  // Returns empty trajectory if none exists.
  // NOTE: this function updates 'traj_nodes_' and 'traj_node_times_'.
  // These variables are needed to keep track of the most recently delivered
  // trajectory to the planner manager; it is used here to determine which
  // node the new trajectory should start from.
  Trajectory<S> ExtractTrajectory() const;

  // Update cost to come, best parent, time, and all traj_to_child times
  // recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateDescendants(const typename Node::Ptr& node) const;

  // Update cost to home and best home child recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateAncestorsOnHome(const typename Node::Ptr& node) const;

  // Update cost to goal and best goal child recursively.
  // NOTE: this will never get into an infinite loop because eventually
  // every node will know its best option and reject further updates.
  void UpdateAncestorsOnGoal(const typename Node::Ptr& node) const;

  // Number of neighbors and radius to use for nearest neighbor searches.
  size_t num_neighbors_;
  double search_radius_;

  // Epsilon greedy exploration parameter. This is the probability of sampling
  // a random (viable!) state to visit.
  double epsilon_greedy_;
  mutable std::random_device rd_;
  mutable std::default_random_engine rng_;

  // Unordered set storing nodes that we have not yet visited.
  mutable std::unordered_set<typename Node::Ptr> nodes_to_visit_;

  // Goal node. This will be set on the first planning invocation.
  mutable typename Node::Ptr goal_node_;

  // Backward-reachable set of the "home" state.
  // This is the initial state of the planner which we assume is viable.
  mutable std::unique_ptr<SearchableSet<Node, S>> home_set_;

  // Parallel lists of nodes and times.
  // These correspond to the most recently output trajectory and are used
  // for quickly identifying query start states on the graph.
  // NOTE: these times are absolute ROS times, not relative to 0.0.
  // NOTE: we also store the index of the node we selected for exploration,
  // since
  // we want to make sure we always get there if we replan.
  mutable std::vector<typename Node::Ptr> traj_nodes_;
  mutable std::vector<double> traj_node_times_;
  mutable size_t explore_node_idx_;

  // Publisher and topic for visualization. Also fixed frame name and colormap.
  ros::Publisher vis_pub_;
  std::string vis_topic_;
  std::string fixed_frame_;

  mutable Colormap colormap_;
};  //\class GraphDynamicPlanner

// ----------------------------- IMPLEMENTATION ----------------------------- //

// Plan a trajectory from the given start to goal states starting
// at the given time.o
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::Plan(
    const S& start, const S& goal, double start_time) const {
  // Keep track of initial time.
  const double initial_call_time = ros::Time::now().toSec();

  // Set up goal node.
  if (!goal_node_) {
    goal_node_ = Node::Create();
    goal_node_->state = goal;
    goal_node_->time = constants::kInfinity;
    goal_node_->cost_to_come = constants::kInfinity;
    goal_node_->cost_to_home = constants::kInfinity;
    goal_node_->cost_to_goal = 0.0;
    goal_node_->is_viable = true;
    goal_node_->is_visited = false;
  } else {
    // Make sure the goal has not changed.
    if (!goal_node_->state.ToVector().isApprox(goal.ToVector(),
                                               constants::kEpsilon)) {
      throw std::runtime_error("Oops. Goal changed.");
    }
  }

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
    home_node->is_visited = true;
    home_node->time = 0.0;

    home_set_.reset(new SearchableSet<Node, S>(home_node));
    start_node = home_node;

    // Update colormap.
    colormap_.UpdateTimes(home_node->time);
  }

  // Generate trajectory. This trajectory will originate from the start node and
  // either terminate within the goal set OR at the start_node and pass through
  // the home node.
  const Trajectory<S> traj = RecursivePlan(initial_call_time, true);

  // Visualize the new graph.
  Visualize();

  // NOTE! Don't need to sleep until max runtime is exceeded because we're going
  // to include the trajectory segment the planner is already on.
  return traj;
}

// Recursive version of Plan() that plans outbound and return trajectories.
// High level recursive feasibility logic is here.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::RecursivePlan(
    double initial_call_time, bool outbound) const {
  // Loop until we run out of time.
  while (ros::Time::now().toSec() - initial_call_time < this->max_runtime_) {
    // (1) Sample a new point.
    const S sample = S::Sample();

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
          SubPlan(neighboring_parent->state, sample, neighboring_parent->time);
      if (sub_plan.Size() == 0) continue;

      // If somehow the planner returned a plan that does not terminate at the
      // desired goal, or begin at the desired start, then discard.
      if (!CheckTrajectoryEndpoints(sub_plan, neighboring_parent->state,
                                    sample))
        continue;

      // Add to graph.
      sample_node = Node::Create();
      sample_node->state = sample;
      sample_node->time = neighboring_parent->time + sub_plan.Duration();
      sample_node->cost_to_come =
          neighboring_parent->cost_to_come + Cost(sub_plan);
      sample_node->cost_to_home = constants::kInfinity;
      sample_node->is_viable = false;
      sample_node->is_visited = false;
      sample_node->best_parent = neighboring_parent;
      sample_node->best_home_child = nullptr;
      sample_node->parents = {neighboring_parent};
      sample_node->trajs_to_children = {};

      // Update colormap.
      colormap_.UpdateTimes(sample_node->time);

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
        (outbound) ? std::vector<typename Node::Ptr>({goal_node_})
                   : home_set_->KnnSearch(sample, num_neighbors_);

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

      // Add this guy to 'nodes_to_visit_'.
      nodes_to_visit_.emplace(sample_node);
      break;
    }

    if (child == nullptr) {
      // (5) If outbound, make a recursive call. We can ignore the returned
      // trajectory since we'll generate one later once we're all done.
      if (outbound) {
        const Trajectory<S> ignore = RecursivePlan(initial_call_time, false);
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
            child->cost_to_goal +
            Cost(sample_node->trajs_to_children.at(child));

        sample_node->best_goal_child = child;

        // Propagate backward to all ancestors.
        UpdateAncestorsOnGoal(sample_node);
      } else {
        // Update sample node's cost to home.
        sample_node->cost_to_home =
            child->cost_to_home +
            Cost(sample_node->trajs_to_children.at(child));

        sample_node->best_home_child = child;

        // Propagate backward to all ancestors.
        UpdateAncestorsOnHome(sample_node);
      }

      // Extract trajectory. Always walk backward from the initial node of the
      // goal set to the start node. Be sure to set the correct start time.
      // NOTE: this will automatically set traj_nodes and traj_node_times.
      // Else, return a dummy trajectory since it will be ignored anyway.
      if (outbound) {
        return ExtractTrajectory();
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
  const auto traj = ExtractTrajectory();

  std::cout << "Traj initial time - now: "
            << traj.FirstTime() - ros::Time::now().toSec() << std::endl;
  return traj;
}

// Extract a trajectory including the given start time, which either
// loops back home or goes to the goal (if such a trajectory exists).
// Returns empty trajectory if none exists.
// NOTE: this function updates 'traj_nodes_' and 'traj_node_times_'.
// These variables are needed to keep track of the most recently delivered
// trajectory to the planner manager; it is used here to determine which
// node the new trajectory should start from.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
Trajectory<S> GraphDynamicPlanner<S, E, D, SD, B, SB>::ExtractTrajectory()
    const {
  // Start node for graph traversal. This will either be the home node, or
  // the next node along our previous plan.
  typename Node::Ptr start_node = nullptr;

  // Accumulate trajectories and nodes in a list.
  std::list<Trajectory<S>> trajs;
  std::list<typename Node::Ptr> nodes;

  // Time to use for first node in trajectory.
  double first_traj_time = ros::Time::now().toSec();

  // If we have traj_nodes and traj_node_times, interpolate at the current time
  // and reset to include only the node/time preceding the current_time.
  // The node/time succeeding the current time will be added later in this
  // method.
  if (traj_nodes_.empty()) {
    start_node = home_set_->InitialNode();
    explore_node_idx_ = 0;
  } else {
    const auto iter = std::lower_bound(traj_node_times_.begin(),
                                       traj_node_times_.end(), first_traj_time);
    if (iter == traj_node_times_.end()) {
      throw std::runtime_error("Invalid start time: too late.");
    }

    if (iter == traj_node_times_.begin()) {
      throw std::runtime_error("Invalid start time: too early.");
    }

    size_t hi = std::distance(traj_node_times_.begin(), iter);
    const size_t lo = hi - 1;
    hi = std::max(hi, explore_node_idx_);

    // Remove all nodes up to and including previous node from nodes to visit,
    // and also be sure to mark them as visited.
    for (size_t ii = 0; ii <= lo; ii++) {
      auto& visited_node = traj_nodes_[ii];

      // This will erase only if it was there to start with.
      nodes_to_visit_.erase(visited_node);

      // Mark as visited.
      visited_node->is_visited = true;
    }

    // Extract nodes and times.
    //    const auto& previous_node = traj_nodes_[lo];
    start_node = traj_nodes_[hi];
    first_traj_time = traj_node_times_[lo];

    // Extract trajectory from previous node to start node and ensure
    // that it begins at the right time.
    for (size_t ii = lo; ii < hi; ii++) {
      const auto& node = traj_nodes_[ii];
      const auto& next_node = traj_nodes_[ii + 1];
      const auto& traj = node->trajs_to_children.at(next_node);
      trajs.push_back(traj);
      nodes.push_back(node);
    }

    // Update 'explore_node_idx_' to account for the nodes we've removed from
    // 'traj_nodes_'.
    explore_node_idx_ =
        std::max(0, static_cast<int>(explore_node_idx_) - static_cast<int>(lo));
  }

  // If we are connected to the goal (i.e. we have a best goal child),
  // then just follow best goal child all the way there!
  if (start_node->best_goal_child) {
    nodes.push_back(start_node);
    for (auto node = start_node; node->cost_to_goal > 0.0;
         node = node->best_goal_child) {
      trajs.push_back(node->trajs_to_children.at(node->best_goal_child));
      nodes.push_back(node->best_goal_child);
    }
  } else {
    // We're going home.
    // (1) Follow best home child till we get home.
    // (2) Pick a optimistic node from 'nodes_to_visit_'.
    // (3) Backtrack from that node all the way home via best parent.
    // (4) From that node, follow best home child all the way home.
    // (5) Stitch these three sub-trajectories together in the right order:
    //     { start_node -> home -> optimistic new node -> home }.

    // (1) Follow best home child till we get home.
    nodes.push_back(start_node);
    for (auto node = start_node; node->cost_to_home > 0.0;
         node = node->best_home_child) {
      trajs.push_back(node->trajs_to_children.at(node->best_home_child));
      nodes.push_back(node->best_home_child);
    }

    // (2) Pick a optimistic node from 'nodes_to_visit_'.
    // Choose the one with best heuristic value.
    if (nodes_to_visit_.empty()) {
      // We've explored the entire space and there is no way to the goal.
      // So, just return home and give up.
      ROS_WARN("%s: There is no recursively feasible trajectory to the goal",
               this->name_.c_str());
      ROS_WARN("%s: The safely reachable space has been fully explored.",
               this->name_.c_str());
      ROS_WARN("%s: Returning home.", this->name_.c_str());
    } else {
      // Take a uniform random draw from [0, 1] and if it is below
      // 'epsilon_greedy_' choose a random element. Otherwise choose at random.
      std::uniform_real_distribution<double> unif(0.0, 1.0);
      auto iter =
          (unif(rng_) < epsilon_greedy_)
              ? nodes_to_visit_.begin()
              : std::min_element(nodes_to_visit_.begin(), nodes_to_visit_.end(),
                                 [this](const typename Node::Ptr& node1,
                                        const typename Node::Ptr& node2) {
                                   return Heuristic(node1->state) <
                                          Heuristic(node2->state);
                                 });
      auto new_node_to_visit = *iter;
      nodes_to_visit_.erase(iter);

      // (3) Backtrack from that node all the way home via best parent.
      std::list<typename Node::Ptr> backward_nodes;
      std::list<Trajectory<S>> backward_trajs;
      for (auto node = new_node_to_visit; node->cost_to_come > 0.0;
           node = node->best_parent) {
        backward_trajs.push_front(
            node->best_parent->trajs_to_children.at(node));
        backward_nodes.push_front(node);
      }

      // Append these nodes and trajs to the main lists.
      trajs.insert(trajs.end(), backward_trajs.begin(), backward_trajs.end());
      nodes.insert(nodes.end(), backward_nodes.begin(), backward_nodes.end());

      // Only update 'explore_node_idx_' if we've reached it.
      if (explore_node_idx_ == 0) explore_node_idx_ = nodes.size() - 1;

      // (4) From that node, follow best home child all the way home.
      for (auto node = new_node_to_visit; node->cost_to_home > 0.0;
           node = node->best_home_child) {
        trajs.push_back(node->trajs_to_children.at(node->best_home_child));
        nodes.push_back(node->best_home_child);
      }

      // (5) Stitch these three sub-trajectories together in the right order:
      //     { start_node -> home -> optimistic new node -> home }.
      // Already done. Oh man.
    }
  }

  // Reinitialize traj nodes/times with previous node/time. Start node/time
  // will be added later.
  traj_nodes_.clear();
  traj_node_times_.clear();

  // Add all nodes and corresponding times to traj_nodes and traj_node_times.
  double node_time = first_traj_time;
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
  traj.ResetFirstTime(first_traj_time);
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

  // Visualization parameters.
  if (!nl.getParam("vis/graph", vis_topic_)) return false;
  if (!nl.getParam("frame/fixed", fixed_frame_)) return false;

  // Epsilon for epsilon-greedy exploration.
  if (!nl.getParam("epsilon_greedy", epsilon_greedy_)) return false;

  return true;
}

// Register callbacks.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
bool GraphDynamicPlanner<S, E, D, SD, B, SB>::RegisterCallbacks(
    const ros::NodeHandle& n) {
  if (!Planner<S, E, D, SD, B, SB>::RegisterCallbacks(n)) return false;

  ros::NodeHandle nl(n);
  vis_pub_ =
      nl.advertise<visualization_msgs::Marker>(vis_topic_.c_str(), 1, false);

  return true;
}

// Visualize the graph.
template <typename S, typename E, typename D, typename SD, typename B,
          typename SB>
void GraphDynamicPlanner<S, E, D, SD, B, SB>::Visualize() const {
  if (vis_pub_.getNumSubscribers() == 0) {
    ROS_WARN_THROTTLE(1.0, "%s: I'm lonely. Please subscribe.",
                      this->name_.c_str());
    return;
  }

  if (!home_set_) {
    ROS_ERROR_THROTTLE(1.0, "%s: Tried to visualize without a home set.",
                       this->name_.c_str());
    return;
  }

  // Set up spheres marker. This will be used to mark non-viable nodes.
  visualization_msgs::Marker spheres;
  spheres.ns = "spheres";
  spheres.header.frame_id = fixed_frame_;
  spheres.header.stamp = ros::Time::now();
  spheres.id = 0;
  spheres.type = visualization_msgs::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::Marker::ADD;
  spheres.scale.x = 0.2;
  spheres.scale.y = 0.2;
  spheres.scale.z = 0.2;

  // Set up cubes marker. This will be used to mark viable nodes.
  visualization_msgs::Marker cubes;
  cubes.ns = "cubes";
  cubes.header.frame_id = fixed_frame_;
  cubes.header.stamp = ros::Time::now();
  cubes.id = 0;
  cubes.type = visualization_msgs::Marker::CUBE_LIST;
  cubes.action = visualization_msgs::Marker::ADD;
  cubes.scale.x = 0.2;
  cubes.scale.y = 0.2;
  cubes.scale.z = 0.2;

  // Set up line list marker. This will be used to mark all edges in the graph.
  visualization_msgs::Marker lines;
  lines.ns = "lines";
  lines.header.frame_id = fixed_frame_;
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.1;

  // Walk the graph via breadth-first search.
  std::unordered_set<typename Node::Ptr> visited_nodes;
  std::list<typename Node::Ptr> nodes_to_expand({home_set_->InitialNode()});

  while (!nodes_to_expand.empty()) {
    const auto current_node = nodes_to_expand.front();
    nodes_to_expand.pop_front();

    // Add to visited list.
    visited_nodes.emplace(current_node);

    // Add to sphere marker.
    geometry_msgs::Point current_position;
    current_position.x = current_node->state.X();
    current_position.y = current_node->state.Y();
    current_position.z = current_node->state.Z();

    auto current_color = colormap_(current_node->time);
    auto& node_marker = (current_node->is_viable) ? cubes : spheres;
    node_marker.points.push_back(current_position);
    node_marker.colors.push_back(current_color);

    // Lower alpha value for color in order to make lines show up translucent.
    constexpr double kTranslucentAlpha = 0.1;
    current_color.a = kTranslucentAlpha;

    // Expand this node.
    for (const auto& child_traj_pair : current_node->trajs_to_children) {
      const auto& child = child_traj_pair.first;

      // Add to queue if unvisited.
      if (visited_nodes.count(child)) nodes_to_expand.push_back(child);

      // Publish lines only.
      geometry_msgs::Point child_position;
      child_position.x = child->state.X();
      child_position.y = child->state.Y();
      child_position.z = child->state.Z();

      auto child_color = colormap_(child->time);
      child_color.a = kTranslucentAlpha;
      lines.points.push_back(current_position);
      lines.points.push_back(child_position);
      lines.colors.push_back(current_color);
      lines.colors.push_back(child_color);
    }
  }

  // Publish.
  vis_pub_.publish(spheres);
  vis_pub_.publish(cubes);
  vis_pub_.publish(lines);
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

        // Update colormap.
        colormap_.UpdateTimes(child->time);

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
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateAncestorsOnHome(
    const typename Node::Ptr& node) const {
  // Run breadth-first search.
  // Initialize a queue with 'node' inside.
  std::list<typename Node::Ptr> queue = {node};

  while (queue.size() > 0) {
    // Pop oldest node.
    const typename Node::Ptr current_node = queue.front();
    queue.pop_front();

    // Loop over all parents and add to queue if this node is their
    // NEW best home child.
    for (auto& parent : current_node->parents) {
      // Parent is DEFINITELY viable.
      parent->is_viable = true;

      // If parent has not been visited yet, then add to 'nodes_to_visit_'.
      if (!parent->is_visited && !nodes_to_visit_.count(parent))
        nodes_to_visit_.emplace(parent);

      // Extract trajectory from parent to this node.
      auto& traj_from_parent = parent->trajs_to_children.at(current_node);

      // Maybe update parent's best home child to be the current node.
      // If so, also update its viability and cost to home.
      // NOTE: if parent's cost to home is infinite, we are definitely its
      // best
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
void GraphDynamicPlanner<S, E, D, SD, B, SB>::UpdateAncestorsOnGoal(
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

      // If parent has not been visited yet, then add to 'nodes_to_visit_'.
      if (!parent->is_visited && !nodes_to_visit_.count(parent))
        nodes_to_visit_.emplace(parent);

      // Extract trajectory from parent to this node.
      auto& traj_from_parent = parent->trajs_to_children.at(current_node);

      // Maybe update parent's best goal child to be the current node.
      // If so, also update its viability and cost to goal.
      // NOTE: if parent's cost to goal is infinite, we are definitely its
      // best
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
