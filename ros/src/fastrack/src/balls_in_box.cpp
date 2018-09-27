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
// BallsInBox is derived from the Environment base class. This class models
// obstacles as spheres to provide a simple demo.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/environment/balls_in_box.h>

namespace fastrack {
namespace environment {

// Derived classes must provide a collision checker which returns true if
// and only if the provided position is a valid collision-free configuration.
// Ignores 'time' since this is a time-invariant environment.
bool BallsInBox::IsValid(const Vector3d& position, const TrackingBound& bound,
                         double time) const {
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check an uninitialized BallsInBox.",
             name_.c_str());
    return false;
  }

  // Check that this position is within the outer environment boundaries.
  if (!bound.ContainedWithinBox(position, lower_, upper_)) return false;

  // Check against each obstacle.
  // NOTE! Just using a linear search here for simplicity.
  if (centers_.size() > 100) {
    ROS_WARN_THROTTLE(1.0,
                      "%s: Caution! Linear search may be slowing you down.",
                      name_.c_str());
  }

  for (size_t ii = 0; ii < centers_.size(); ii++) {
    if (bound.OverlapsSphere(position, centers_[ii], radii_[ii])) return false;
  }

  return true;
}

// Update this environment with the information contained in the given
// sensor measurement.
// NOTE! This function needs to publish on `updated_topic_`.
void BallsInBox::SensorCallback(
    const fastrack_msgs::SensedSpheres::ConstPtr& msg) {
  // Check list lengths.
  if (msg->centers.size() != msg->radii.size())
    ROS_WARN("%s: Malformed SensedSpheres msg.", name_.c_str());

  const size_t num_obstacles = std::min(msg->centers.size(), msg->radii.size());

  // Add each unique obstacle to list.
  // NOTE! Just using a linear search here for simplicity.
  if (centers_.size() > 100)
    ROS_WARN_THROTTLE(1.0,
                      "%s: Caution! Linear search may be slowing you down.",
                      name_.c_str());

  bool any_unique = false;
  for (size_t ii = 0; ii < num_obstacles; ii++) {
    const Vector3d p(msg->centers[ii].x, msg->centers[ii].y,
                     msg->centers[ii].z);
    const double r = msg->radii[ii];

    // If not unique, discard.
    bool unique = true;
    for (size_t jj = 0; jj < centers_.size(); jj++) {
      if (p.isApprox(centers_[jj], constants::kEpsilon) &&
          std::abs(r - radii_[jj]) < constants::kEpsilon) {
        unique = false;
        break;
      }
    }

    if (unique) {
      any_unique = true;
      centers_.push_back(p);
      radii_.push_back(r);
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
fastrack_msgs::SensedSpheres BallsInBox::SimulateSensor(
    const SphereSensorParams& params) const {
  // Set up msg.
  fastrack_msgs::SensedSpheres msg;
  msg.sensor_position.x = params.position(0);
  msg.sensor_position.y = params.position(1);
  msg.sensor_position.z = params.position(2);
  msg.sensor_radius = params.range;

  // Check each obstacle and, if in range, add to response.
  geometry_msgs::Vector3 c;
  for (size_t ii = 0; ii < centers_.size(); ii++) {
    if ((params.position - centers_[ii]).norm() < params.range + radii_[ii]) {
      c.x = centers_[ii](0);
      c.y = centers_[ii](1);
      c.z = centers_[ii](2);

      msg.centers.push_back(c);
      msg.radii.push_back(radii_[ii]);
    }
  }

  return msg;
}

// Derived classes must have some sort of visualization through RViz.
void BallsInBox::Visualize() const {
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
  for (size_t ii = 0; ii < centers_.size(); ii++) {
    visualization_msgs::Marker sphere;
    sphere.ns = "sphere";
    sphere.header.frame_id = fixed_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.id = static_cast<int>(ii);
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.scale.x = 2.0 * radii_[ii];
    sphere.scale.y = 2.0 * radii_[ii];
    sphere.scale.z = 2.0 * radii_[ii];

    sphere.color.a = 0.9;
    sphere.color.r = 0.7;
    sphere.color.g = 0.5;
    sphere.color.b = 0.5;

    geometry_msgs::Point p;
    const Vector3d point = centers_[ii];
    p.x = point(0);
    p.y = point(1);
    p.z = point(2);

    sphere.pose.position = p;

    // Publish sphere marker.
    vis_pub_.publish(sphere);
    ros::Duration(0.01).sleep();
  }
}

// Load parameters. This may be overridden by derived classes if needed
// (they should still call this one via Environment::LoadParameters).
bool BallsInBox::LoadParameters(const ros::NodeHandle& n) {
  if (!Environment<fastrack_msgs::SensedSpheres,
                   SphereSensorParams>::LoadParameters(n))
    return false;

  ros::NodeHandle nl(n);

  // Pre-specified obstacles.
  std::vector<double> obstacle_xs, obstacle_ys, obstacle_zs, obstacle_rs;
  if (nl.getParam("env/obstacle/xs", obstacle_xs) &&
      nl.getParam("env/obstacle/ys", obstacle_ys) &&
      nl.getParam("env/obstacle/zs", obstacle_zs) &&
      nl.getParam("env/obstacle/rs", obstacle_rs)) {
    if (obstacle_xs.size() != obstacle_ys.size() ||
        obstacle_zs.size() != obstacle_rs.size() ||
        obstacle_ys.size() != obstacle_zs.size()) {
      ROS_WARN("%s: Pre-specified obstacles are malformed.", name_.c_str());
    } else {
      for (size_t ii = 0; ii < obstacle_xs.size(); ii++) {
        centers_.emplace_back(
            Vector3d(obstacle_xs[ii], obstacle_ys[ii], obstacle_zs[ii]));
        radii_.push_back(obstacle_rs[ii]);
      }
    }
  } else {
    // No pre-specified obstacles.
    ROS_INFO("%s: No pre-specified obstacles.", name_.c_str());
  }

  // Number/size of random obstacles.
  int num;
  double min_radius, max_radius;
  if (!nl.getParam("env/num_random_obstacles", num)) return false;
  if (!nl.getParam("env/min_radius", min_radius)) return false;
  if (!nl.getParam("env/max_radius", max_radius)) return false;

  // Random seed.
  int seed;
  if (!nl.getParam("env/seed", seed)) return false;

  // Generate obstacles.
  GenerateObstacles(static_cast<size_t>(num), min_radius, max_radius, seed);

  return true;
}

// Generate random obstacles.
void BallsInBox::GenerateObstacles(size_t num, double min_radius,
                                   double max_radius, unsigned int seed) {
  // Create a random number generator.
  std::random_device rd;
  std::default_random_engine rng(rd());
  rng.seed(seed);

  // Set up x/y/z and radius distributions.
  std::uniform_real_distribution<double> unif_x(lower_(0), upper_(0));
  std::uniform_real_distribution<double> unif_y(lower_(1), upper_(1));
  std::uniform_real_distribution<double> unif_z(lower_(2), upper_(2));
  std::uniform_real_distribution<double> unif_r(min_radius, max_radius);

  // Create obstacles.
  for (size_t ii = 0; ii < num; ii++) {
    centers_.push_back(Vector3d(unif_x(rng), unif_y(rng), unif_z(rng)));
    radii_.push_back(unif_r(rng));
  }
}

}  //\namespace environment
}  //\namespace fastrack
