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

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>
#include <flann/flann.h>

namespace fastrack {
namespace planning {

template<typename N, typename S>
class SearchableSet : private Uncopyable {
public:
  ~SearchableSet();
  explicit SearchableSet(const typename N::ConstPtr& node);

  // Access the initial node.
  inline typename N::ConstPtr InitialNode() const { return registry_.front(); }

  // Insert a new node into the set.
  bool Insert(const typename N::ConstPtr& node);

  // Nearest neighbor search.
  std::vector<typename N::ConstPtr> KnnSearch(const S& query, size_t k) const;

  // Radius search.
  std::vector<typename N::ConstPtr> RadiusSearch(const S& query, double r) const;

private:
  // A Flann kdtree. Searches in this index return indices, which are then mapped
  // to node pointers in an array.
  // TODO: fix the distance metric to be something more intelligent.
  std::unique_ptr< flann::KDTreeIndex< flann::L2<double> > > index_;
  std::vector<typename N::ConstPtr> registry_;
};

// ------------------------------ IMPLEMENTATION ----------------------------- //

// Destructor. Must free all memory in the index.
template<typename N, typename S>
SearchableSet<N, S>::~SearchableSet() {
  // Free memory from points in the kdtree.
  if (index_ != nullptr) {
    for (size_t ii = 0; ii < index_->size(); ++ii) {
      double* point = index_->getPoint(ii);
      delete[] point;
    }
  }
}

// Construct from a single node.
template<typename N, typename S>
SearchableSet<N, S>::SearchableSet(const typename N::ConstPtr& node) {
  if (!node.get()) {
    ROS_WARN("SearchableSet: Constructing without initial node.");
  } else {
    Insert(node);
  }
}

// Insert a new node into the set.
template<typename N, typename S>
bool SearchableSet<N, S>::Insert(const typename N::ConstPtr& node) {
  if (!node.get()) {
    ROS_WARN("SearchableSet: Tried to insert a null node.");
    return false;
  }

  // Copy the input point into FLANN's Matrix type.
  const VectorXd x = node->state.ToVector();
  flann::Matrix<double> flann_point(new double[x.size()], 1, x.size());

  for (size_t ii = 0; ii < x.size(); ii++)
    flann_point[0][ii] = x(ii);

  // If this is the first point in the index, create the index and exit.
  if (index_ == nullptr) {
    // Single kd-tree.
    const int kNumTrees = 1;
    index_.reset(new flann::KDTreeIndex< flann::L2<double> >(
      flann_point, flann::KDTreeIndexParams(kNumTrees)));

    index_->buildIndex();
  } else {
    // If the index is already created, add the data point to the index.
    // Rebuild every time the index floats in size to occasionally rebalance
    // the kdtree.
    const double kRebuildThreshold = 2.0;
    index_->addPoints(flann_point, kRebuildThreshold);
  }

  // Add point to registry.
  registry_.push_back(node);

  return true;
}

// Nearest neighbor search.
template<typename N, typename S>
std::vector<typename N::ConstPtr> SearchableSet<N, S>::
KnnSearch(const S& query, size_t k) const {
  if (index_ == nullptr) {
    ROS_WARN("SearchableSet: Cannot search empty index.");
    return std::vector<typename N::ConstPtr>();
  }

  // Convert the input point to the FLANN format.
  const VectorXd x = query.ToVector();
  const flann::Matrix<double> flann_query(x.data(), 1, x.size());

  // Search the kd tree for the nearest neighbor to the query.
  std::vector< std::vector<int> > query_match_indices;
  std::vector< std::vector<double> > query_squared_distances;

  const int num_neighbors_found =
    index_->knnSearch(flann_query, query_match_indices,
                      query_squared_distances, static_cast<int>(k),
                      flann::SearchParams(-1, 0.0, false));

  // Assign output.
  std::vector<typename N::ConstPtr> neighbors;
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[ query_match_indices[0][ii] ]);

  return neighbors;
}

// Radius search.
template<typename N, typename S>
std::vector<typename N::ConstPtr> SearchableSet<N, S>::
RadiusSearch(const S& query, double r) const {
  if (index_ == nullptr) {
    ROS_WARN("SearchableSet: cannot search empty index.");
    return std::vector<typename N::ConstPtr>();
  }

  // Convert the input point to the FLANN format.
  const VectorXd x = query.ToVector();
  const flann::Matrix<double> flann_query(x.data(), 1, x.size());

  // Search the kd tree for the nearest neighbor to the query.
  std::vector< std::vector<int> > query_match_indices;
  std::vector< std::vector<double> > query_squared_distances;

  // FLANN checks Euclidean distance squared, so we pass in r * r.
  int num_neighbors_found =
    index_->radiusSearch(flann_query, query_match_indices,
                         query_squared_distances, r * r,
                         flann::SearchParams(-1, 0.0, false));
  // Assign output.
  std::vector<typename N::ConstPtr> neighbors;
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[ query_match_indices[0][ii] ]);

  return neighbors;
}

} //\namespace planning
} //\namespace fastrack

#endif
