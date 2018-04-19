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
// Defines SearchableSet class, which is a wrapper around the FLANN library's
// fast kdtree index. SearchableSets are templated on the type of
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_FLANN_TREE_H
#define META_PLANNER_FLANN_TREE_H

#include <meta_planner/waypoint.h>
#include <utils/types.h>
#include <utils/uncopyable.h>

#include <ros/ros.h>
#include <flann/flann.h>
#include <memory>
#include <vector>
#include <math.h>

namespace meta {

class FlannTree : private Uncopyable {
public:
  explicit FlannTree() {}
  ~FlannTree();

  // Insert a new Waypoint into the tree.
  bool Insert(const Waypoint::ConstPtr& waypoint);

  // Nearest neighbor search.
  std::vector<Waypoint::ConstPtr> KnnSearch(Vector3d& query, size_t k) const;

  // Radius search.
  std::vector<Waypoint::ConstPtr> RadiusSearch(Vector3d& query, double r) const;

private:
  // A Flann kdtree. Searches in this tree return indices, which are then mapped
  // to Waypoint pointers in an array.
  // TODO: fix the distance metric to be something more intelligent.
  std::unique_ptr< flann::KDTreeIndex< flann::L2<double> > > index_;
  std::vector<Waypoint::ConstPtr> registry_;
};

} //\namespace meta

#endif
