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
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/environment/balls_in_box_occupancy_map.h>

namespace fastrack {
namespace environment {

// Occupancy probability for a single point.
double BallsInBoxOccupancyMap::OccupancyProbability(const Vector3d& p) const {
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check without initializing.",
             name_.c_str());
    return kOccupiedProbability;
  }

  // Check box limits.
  if (p(0) < lower_(0) || p(0) > upper_(0) || p(1) < lower_(1) ||
      p(1) > upper_(1) || p(2) < lower_(2) || p(2) > upper_(2))
    return kOccupiedProbability;

  // Check if this point is inside any obstacles.
  // NOTE: use KnnSearch instead of RadiusSearch for extra precision.
  constexpr size_t kOneNearestNeighbor = 1;
  const std::vector<std::pair<Vector3d, double>> neighboring_obstacles =
      obstacles_.KnnSearch(p, kOneNearestNeighbor);
  for (const auto& entry : neighboring_obstacles) {
    if ((p - entry.first).norm() < entry.second) return kOccupiedProbability;
  }

  // Check if this point is inside any sensor FOVs.
  const std::vector<std::pair<Vector3d, double>> neighboring_sensors =
      sensor_fovs_.KnnSearch(p, kOneNearestNeighbor);
  for (const auto& entry : neighboring_sensors) {
    if ((p - entry.first).norm() < entry.second) return kFreeProbability;
  }

  return kUnknownProbability;
}

// Occupancy probability for a box tracking error bound centered at the given
// point. Occupancy is set to occupied if ANY of the box is occupied. Next,
// if ANY of the box is unknown the result is unknown. Otherwise, free.
double BallsInBoxOccupancyMap::OccupancyProbability(const Vector3d& p,
                                                    const Box& bound) const {
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check without initializing.",
             name_.c_str());
    return kOccupiedProbability;
  }

  // Check box limits.
  if (p(0) < lower_(0) + bound.x || p(0) > upper_(0) - bound.x ||
      p(1) < lower_(1) + bound.y || p(1) > upper_(1) - bound.y ||
      p(2) < lower_(2) + bound.z || p(2) > upper_(2) - bound.z)
    return kOccupiedProbability;

  // Helper function to check if any sphere in the given KdtreeMap overlaps
  // the given Box.
  auto overlaps = [&p, &bound](const KdtreeMap<3, double>& kdtree,
                               double largest_radius) {
    // Get nearest neighbors.
    // NOTE: using KnnSearch instead of RadiusSearch because it seems to
    // be more precise.
    constexpr size_t kOneNearestNeighbor = 1;
    const auto neighbors = kdtree.KnnSearch(p, kOneNearestNeighbor);

    // Check for overlaps.
    const Vector3d bound_vector(bound.x, bound.y, bound.z);
    for (const auto& entry : neighbors) {
      // Find closest point to neighbor within bound.
      Vector3d closest_point;
      for (size_t ii = 0; ii < 3; ii++) {
        closest_point(ii) =
            std::min(p(ii) + bound_vector(ii),
                     std::max(p(ii) - bound_vector(ii), entry.first(ii)));
      }

      // Is closest point within radius.
      if ((closest_point - entry.first).norm() <= entry.second) return true;
    }

    return false;
  };  //\overlaps

  // Check if this point is inside any obstacles.
  if (overlaps(obstacles_, largest_obstacle_radius_))
    return kOccupiedProbability;

  // Check if this point contains any unknown space.
  if (!overlaps(sensor_fovs_, largest_sensor_radius_))
    return kUnknownProbability;

  return kFreeProbability;
}

// Update this environment with the information contained in the given
// sensor measurement.
// NOTE! This function needs to publish on `updated_topic_`.
void BallsInBoxOccupancyMap::SensorCallback(
    const fastrack_msgs::SensedSpheres::ConstPtr& msg) {
  // Add sensor FOV to kdtree.
  sensor_fovs_.Insert(
      std::make_pair(Vector3d(msg->sensor_position.x, msg->sensor_position.y,
                              msg->sensor_position.z),
                     msg->sensor_radius));

  // Check list lengths.
  if (msg->centers.size() != msg->radii.size())
    ROS_WARN("%s: Malformed SensedSpheres msg.", name_.c_str());

  const size_t num_obstacles = std::min(msg->centers.size(), msg->radii.size());

  // Add each unique obstacle to list.
  bool any_unique = false;
  for (size_t ii = 0; ii < num_obstacles; ii++) {
    const Vector3d p(msg->centers[ii].x, msg->centers[ii].y,
                     msg->centers[ii].z);
    const double r = msg->radii[ii];

    // If not unique, discard.
    bool unique = true;
    constexpr size_t kOneNearestNeighbor = 1;
    const auto neighbors = obstacles_.KnnSearch(p, kOneNearestNeighbor);
    for (const auto& entry : neighbors) {
      if ((p - entry.first).squaredNorm() < constants::kEpsilon &&
          std::abs(r - entry.second) < constants::kEpsilon) {
        unique = false;
        break;
      }
    }

    if (unique) {
      any_unique = true;
      obstacles_.Insert({p, r});
    }
  }

  if (any_unique) {
    // Let the system know this environment has been updated.
    updated_pub_.publish(std_msgs::Empty());

    // Visualize.
    Visualize();
  }
}

// Generate a sensor measurement as a service response.
fastrack_msgs::SensedSpheres BallsInBoxOccupancyMap::SimulateSensor(
    const SphereSensorParams& params) const {
  fastrack_msgs::SensedSpheres msg;

  // Find nearest neighbors.
  const auto neighbors = obstacles_.RadiusSearch(
      params.position, params.range + largest_obstacle_radius_);

  // Check and see if any are actually in range.
  for (const auto& entry : neighbors) {
    if ((params.position - entry.first).norm() < params.range + entry.second) {
      geometry_msgs::Vector3 c;
      c.x = entry.first(0);
      c.y = entry.first(1);
      c.z = entry.first(2);

      msg.centers.push_back(c);
      msg.radii.push_back(entry.second);
    }
  }

  return msg;
}

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via OccupancyMap::LoadParameters).
bool BallsInBoxOccupancyMap::LoadParameters(const ros::NodeHandle& n) {
  if (!OccupancyMap::LoadParameters(n)) return false;

  return true;
}

// Derived classes must have some sort of visualization through RViz.
void BallsInBoxOccupancyMap::Visualize() const {
  if (vis_pub_.getNumSubscribers() <= 0) return;

  // Set up box marker.
  visualization_msgs::Marker cube;
  cube.ns = "cube";
  cube.header.frame_id = fixed_frame_;
  cube.header.stamp = ros::Time::now();
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.color.a = 0.5;
  cube.color.r = 0.3;
  cube.color.g = 0.7;
  cube.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  cube.scale.x = upper_(0) - lower_(0);
  center.x = lower_(0) + 0.5 * cube.scale.x;

  cube.scale.y = upper_(1) - lower_(1);
  center.y = lower_(1) + 0.5 * cube.scale.y;

  cube.scale.z = upper_(2) - lower_(2);
  center.z = lower_(2) + 0.5 * cube.scale.z;

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish cube marker.
  vis_pub_.publish(cube);

  // Visualize obstacles as spheres.
  const auto& obstacle_registry = obstacles_.Registry();
  for (size_t ii = 0; ii < obstacle_registry.size(); ii++) {
    const auto& entry = obstacle_registry[ii];

    visualization_msgs::Marker sphere;
    sphere.ns = "obstacles";
    sphere.header.frame_id = fixed_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.id = static_cast<int>(ii);
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.scale.x = 2.0 * entry.second;
    sphere.scale.y = 2.0 * entry.second;
    sphere.scale.z = 2.0 * entry.second;

    sphere.color.a = 0.9;
    sphere.color.r = 0.7;
    sphere.color.g = 0.5;
    sphere.color.b = 0.5;

    geometry_msgs::Point p;
    p.x = entry.first(0);
    p.y = entry.first(1);
    p.z = entry.first(2);

    sphere.pose.position = p;

    // Publish sphere marker.
    vis_pub_.publish(sphere);
    ros::Duration(0.001).sleep();
  }

  // Visualize sensor FOVs as spheres too.
  const auto& sensor_registry = sensor_fovs_.Registry();
  for (size_t ii = 0; ii < sensor_registry.size(); ii++) {
    const auto& entry = sensor_registry[ii];

    visualization_msgs::Marker sphere;
    sphere.ns = "sensors";
    sphere.header.frame_id = fixed_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.id = static_cast<int>(ii);
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.scale.x = 2.0 * entry.second;
    sphere.scale.y = 2.0 * entry.second;
    sphere.scale.z = 2.0 * entry.second;

    sphere.color.a = 0.25;
    sphere.color.r = 0.5;
    sphere.color.g = 0.8;
    sphere.color.b = 0.5;

    geometry_msgs::Point p;
    p.x = entry.first(0);
    p.y = entry.first(1);
    p.z = entry.first(2);

    sphere.pose.position = p;

    // Publish sphere marker.
    vis_pub_.publish(sphere);
    ros::Duration(0.001).sleep();
  }
}

}  // namespace environment
}  // namespace fastrack
