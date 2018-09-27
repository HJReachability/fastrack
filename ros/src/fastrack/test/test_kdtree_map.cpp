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
// Unit tests for KdtreeMap.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/utils/kdtree_map.h>
#include <fastrack/utils/types.h>

#include <gtest/gtest.h>
#include <random>
#include <unordered_set>
#include <vector>

namespace {

// Dimension to use for vectors.
static constexpr size_t kNumDimensions = 3;
typedef Eigen::Matrix<double, kNumDimensions, 1> TestVectorType;

// Number of random points to use for tests.
static constexpr size_t kNumRandomInsertions = 100;
static constexpr size_t kNumRandomQueries = 10;

// Random number generator.
static constexpr double kMinValue = -1.0;
static constexpr double kMaxValue = 1.0;
static std::default_random_engine rng;
static std::uniform_real_distribution<double> unif(kMinValue, kMaxValue);

// Utility for generating a random vector.
TestVectorType GenerateRandomVector() {
  TestVectorType p;
  for (size_t jj = 0; jj < kNumDimensions; jj++) p(jj) = unif(rng);
  return p;
}

}  // namespace

TEST(KdtreeMap, TestRadiusSearch) {
  // Generate a bunch of random points, paired with their index in the list.
  std::vector<std::pair<TestVectorType, size_t>> test_points;
  for (size_t ii = 0; ii < kNumRandomInsertions; ii++)
    test_points.emplace_back(GenerateRandomVector(), ii);

  // Insert all these points into the kdtree.
  fastrack::KdtreeMap<kNumDimensions, size_t> kdtree_map;
  ASSERT_TRUE(kdtree_map.Insert(test_points));

  // Radius search a bunch of random points.
  constexpr double kRadius = 0.5 * (kMaxValue - kMinValue);
  constexpr double kAllowedEror = 1.25 * kRadius;
  for (size_t ii = 0; ii < kNumRandomQueries; ii++) {
    const TestVectorType query = GenerateRandomVector();
    const std::vector<std::pair<TestVectorType, size_t>> neighbors =
        kdtree_map.RadiusSearch(query, kRadius);

    // Check through all the points we inserted and make sure that those
    // returned as neighbors are indeed within the radius and that those not
    // returned as neighbors are further away.
    std::unordered_set<size_t> neighbor_indices;
    for (const auto& entry : neighbors) {
      neighbor_indices.emplace(entry.second);
      EXPECT_LE((entry.first - query).norm() - kRadius, kAllowedEror);
      EXPECT_LE((entry.first - test_points[entry.second].first).norm(), 1e-8);
    }

    for (size_t jj = 0; jj < kNumRandomInsertions; jj++) {
      if (neighbor_indices.count(jj)) continue;
      EXPECT_GT((test_points[jj].first - query).norm() - kRadius,
                -kAllowedEror);
    }
  }

  EXPECT_TRUE(true);
}

TEST(KdtreeMap, TestKnnSearch) {
  // Generate a bunch of random points, paired with their index in the list.
  std::vector<std::pair<TestVectorType, size_t>> test_points;
  for (size_t ii = 0; ii < kNumRandomInsertions; ii++)
    test_points.emplace_back(GenerateRandomVector(), ii);

  // Insert all these points into the kdtree.
  fastrack::KdtreeMap<kNumDimensions, size_t> kdtree_map;
  ASSERT_TRUE(kdtree_map.Insert(test_points));

  // Find nearest neighbor for a bunch of random points.
  constexpr size_t kOneNearestNeighbor = 1;
  for (size_t ii = 0; ii < kNumRandomQueries; ii++) {
    const TestVectorType query = GenerateRandomVector();
    const std::vector<std::pair<TestVectorType, size_t>> neighbors =
        kdtree_map.KnnSearch(query, kOneNearestNeighbor);

    ASSERT_EQ(neighbors.size(), kOneNearestNeighbor);

    // Check through all the points we inserted and make sure that all are at
    // least as far away from the query as the neighbor, within an allowed
    // small error.
    constexpr double kAllowedEror =
        0.5 * (kMaxValue - kMinValue) * std::sqrt(kNumDimensions);
    const double neighbor_dist = (neighbors[0].first - query).norm();
    for (size_t jj = 0; jj < kNumRandomInsertions; jj++)
      EXPECT_GE((test_points[jj].first - query).norm() - neighbor_dist,
                -kAllowedEror);
  }

  EXPECT_TRUE(true);
}
