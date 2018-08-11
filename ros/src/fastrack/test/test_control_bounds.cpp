/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
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
// Unit tests for control bounds.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/control/quadrotor_control.h>
#include <fastrack/control/quadrotor_control_bound_box.h>
#include <fastrack/control/quadrotor_control_bound_cylinder.h>
#include <fastrack/control/scalar_bound_interval.h>
#include <fastrack/control/vector_bound_box.h>
#include <fastrack/utils/types.h>

#include <gtest/gtest.h>

using namespace fastrack::control;

namespace {
// Min and max constants.
static constexpr double kBoundMin = -2.0;
static constexpr double kBoundMax = 1.0;

// Dimension to use for vectors.
static constexpr size_t kNumDimensions = 10;

}  // namespace

TEST(ScalarBoundInterval, TestContains) {
  const ScalarBoundInterval bound(std::vector<double>{kBoundMin, kBoundMax});

  // Query a known point.
  constexpr double query_good = 0.0;
  constexpr double query_bad = 2.0;
  EXPECT_TRUE(bound.Contains(query_good));
  EXPECT_FALSE(bound.Contains(query_bad));
}

TEST(ScalarBoundInterval, TestProjectToSurface) {
  const ScalarBoundInterval bound(std::vector<double>{kBoundMin, kBoundMax});

  // Query a known point.
  constexpr double query_pos = 0.1;
  constexpr double query_neg = -0.1;
  EXPECT_EQ(bound.ProjectToSurface(query_pos), kBoundMax);
  EXPECT_EQ(bound.ProjectToSurface(query_neg), kBoundMin);
}

TEST(VectorBoundBox, TestContains) {
  const VectorBoundBox bound(VectorXd::Constant(kNumDimensions, kBoundMin),
                             VectorXd::Constant(kNumDimensions, kBoundMax));

  // Query a known point.
  const VectorXd query_good(VectorXd::Constant(kNumDimensions, 0.0));
  const VectorXd query_bad(VectorXd::Constant(kNumDimensions, 2.0));
  EXPECT_TRUE(bound.Contains(query_good));
  EXPECT_FALSE(bound.Contains(query_bad));
}

TEST(VectorBoundBox, TestProjectToSurface) {
  const VectorBoundBox bound(VectorXd::Constant(kNumDimensions, kBoundMin),
                             VectorXd::Constant(kNumDimensions, kBoundMax));

  // Query separately in each dimension.
  constexpr double kPositiveValue = 0.1;
  VectorXd query(VectorXd::Constant(kNumDimensions, kPositiveValue));
  for (size_t ii = 0; ii < kNumDimensions; ii++) {
    query(ii) = -kPositiveValue;

    const VectorXd projection = bound.ProjectToSurface(query);
    for (size_t jj = 0; jj < kNumDimensions; jj++) {
      if (ii == jj)
        EXPECT_EQ(projection(jj), kBoundMin);
      else
        EXPECT_EQ(projection(jj), kBoundMax);
    }

    query(ii) = kPositiveValue;
  }
}

TEST(QuadrotorControlBoundBox, TestContains) {
  const QuadrotorControl lower(-0.15, -0.15, -1.0, 7.81);
  const QuadrotorControl upper(0.15, 0.15, 1.0, 11.81);
  const QuadrotorControlBoundBox bound(lower, upper);

  // Query a known point.
  const QuadrotorControl query_good(0.0, 0.0, 0.0, 9.81);
  const QuadrotorControl query_bad(-0.1, 0.1, 0.0, 14.0);
  EXPECT_TRUE(bound.Contains(query_good));
  EXPECT_FALSE(bound.Contains(query_bad));
}

TEST(QuadrotorControlBoundBox, TestProjectToSurface) {
  const QuadrotorControl lower(-0.15, -0.15, -1.0, 7.81);
  const QuadrotorControl upper(0.15, 0.15, 1.0, 11.81);
  const QuadrotorControlBoundBox bound(lower, upper);

  // Two queries.
  QuadrotorControl query(0.1, 0.1, 0.1, 10.81);
  QuadrotorControl projection = bound.ProjectToSurface(query);
  EXPECT_EQ(projection.pitch, 0.15);
  EXPECT_EQ(projection.roll, 0.15);
  EXPECT_EQ(projection.yaw_rate, 1.0);
  EXPECT_EQ(projection.thrust, 11.81);

  query.yaw_rate = -0.1;
  projection = bound.ProjectToSurface(query);
  EXPECT_EQ(projection.yaw_rate, -1.0);
}

TEST(QuadrotorControlBoundCylinder, TestContains) {
  constexpr double kPitchRollRadius = 0.15;
  constexpr double kMaxYawRate = 1.0;
  constexpr double kMaxThrustMinusG = 2.0;

  const QuadrotorControlBoundCylinder bound(std::vector<double>{
      kPitchRollRadius, -kMaxYawRate, fastrack::constants::G - kMaxThrustMinusG,
      kMaxYawRate, fastrack::constants::G + kMaxThrustMinusG});

  // Query known points.
  const QuadrotorControl query_good(0.0, 0.0, 0.0, 9.81);
  const QuadrotorControl query_bad(-0.1, 0.1, 0.0, 14.0);
  EXPECT_TRUE(bound.Contains(query_good));
  EXPECT_FALSE(bound.Contains(query_bad));
}

TEST(QuadrotorControlBoundCylinder, TestProjectToSurface) {
  constexpr double kPitchRollRadius = 0.15;
  constexpr double kMaxYawRate = 1.0;
  constexpr double kMaxThrustMinusG = 2.0;

  const QuadrotorControlBoundCylinder bound(std::vector<double>{
      kPitchRollRadius, -kMaxYawRate, fastrack::constants::G - kMaxThrustMinusG,
        kMaxYawRate, fastrack::constants::G + kMaxThrustMinusG});

  // Two queries.
  QuadrotorControl query(0.1, 0.1, 0.1, 10.81);
  QuadrotorControl projection = bound.ProjectToSurface(query);
  EXPECT_NEAR(projection.pitch, kPitchRollRadius * 0.5 * std::sqrt(2.0), 1e-8);
  EXPECT_NEAR(projection.roll, kPitchRollRadius * 0.5 * std::sqrt(2.0), 1e-8);
  EXPECT_EQ(projection.yaw_rate, kMaxYawRate);
  EXPECT_EQ(projection.thrust, fastrack::constants::G + kMaxThrustMinusG);

  query.roll = -0.1;
  projection = bound.ProjectToSurface(query);
  EXPECT_NEAR(projection.roll, -kPitchRollRadius * 0.5 * std::sqrt(2.0), 1e-8);
}
