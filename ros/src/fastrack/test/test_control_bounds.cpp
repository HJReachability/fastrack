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

#include <gtest/gtest.h>

using namespace fastrack::control;

namespace {
// Min and max constants.
static constexpr double kBoundMin = -2.0;
static constexpr double kBoundMax = 1.0;
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
