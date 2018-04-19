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
// fast kdtree index. SearchableSets store a collection of nodes (N) which are
// themselves templated on the state type (S) and allow for nearest neighbor
// and radius searches.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_PLANNING_SEARCHABLE_SET_H
#define FASTRACK_PLANNING_SEARCHABLE_SET_H

#include <utils/types.h>
#include <utils/uncopyable.h>

#include <ros/ros.h>
#include <flann/flann.h>

namespace fastrack {
namespace planning {

template<typename N, typename S>
class SearchableSet : private Uncopyable {
public:
  ~SearchableSet();
  explicit SearchableSet() {}
  explicit SearchableSet(const N::ConstPtr& node);
  explicit SearchableSet(const S& state);

  // Insert a new node into the set.
  bool Insert(const N::ConstPtr& node);

  // Nearest neighbor search.
  std::vector<N::ConstPtr> KnnSearch(const S& query, size_t k) const;

  // Radius search.
  std::vector<N::ConstPtr> RadiusSearch(const S& query, double r) const;

private:
  // A Flann kdtree. Searches in this index return indices, which are then mapped
  // to node pointers in an array.
  // TODO: fix the distance metric to be something more intelligent.
  std::unique_ptr< flann::KDTreeIndex< flann::L2<double> > > index_;
  std::vector<N::ConstPtr> registry_;
};

} //\namespace planning
} //\namespace fastrack

#endif
