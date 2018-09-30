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
// Defines KdtreeMap class, which is a wrapper around the FLANN library's
// fast kdtree index. KdtreeMaps allow for rapid nearest neighbor searches by
// Eigen vector (fixed size) keys and return nearest neighbors as key-value
// pairs.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_UTILS_KDTREE_MAP_H
#define FASTRACK_UTILS_KDTREE_MAP_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <flann/flann.h>
#include <ros/ros.h>

namespace fastrack {

template <int K, typename V>
class KdtreeMap : private Uncopyable {
 public:
  typedef Eigen::Matrix<double, K, 1> VectorKd;

  ~KdtreeMap() {}
  explicit KdtreeMap() {}

  // Insert a new pair into the kdtree.
  bool Insert(const std::pair<VectorKd, V>& entry);
  bool Insert(const VectorKd& key, const V& value) {
    return Insert({key, value});
  }

  // Insert a bunch of entries.
  template <typename Container>
  bool Insert(const Container& pairs) {
    for (const std::pair<VectorKd, V>& entry : pairs)
      if (!Insert(entry)) return false;

    return true;
  }

  // Nearest neighbor search.
  std::vector<std::pair<VectorKd, V>> KnnSearch(const VectorKd& query,
                                                size_t k) const;

  // Radius search.
  std::vector<std::pair<VectorKd, V>> RadiusSearch(const VectorKd& query,
                                                   double r) const;

  // Accessor.
  const std::vector<std::pair<VectorKd, V>>& Registry() const {
    return registry_;
  }
  size_t Size() const { return registry_.size(); }

 private:
  // A Flann kdtree. Searches in this index return indices, which are then
  // mapped to key-value pairs.
  std::unique_ptr<flann::KDTreeIndex<flann::L2<double>>> index_;
  std::vector<std::pair<VectorKd, V>> registry_;
};

// ------------------------------ IMPLEMENTATION -----------------------------

// Insert a new pair into the kdtree.
template <int K, typename V>
bool KdtreeMap<K, V>::Insert(const std::pair<VectorKd, V>& entry) {
  // Append to registry.
  registry_.push_back(entry);

  // Create a FLANN-specific matrix for the key.
  flann::Matrix<double> flann_point(registry_.back().first.data(), 1, K);

  // If this is the first point in the index, create the index and exit.
  if (index_ == nullptr) {
    constexpr int kNumTrees = 1;
    index_.reset(new flann::KDTreeIndex<flann::L2<double>>(
        flann_point, flann::KDTreeIndexParams(kNumTrees)));
    index_->buildIndex();
  } else {
    // If the index is already created, add the data point to the index.
    // Rebuild every time the index doubles in size to occasionally rebalance
    // the kdtree.
    constexpr float kRebuildThreshold = 2.0;
    index_->addPoints(flann_point, kRebuildThreshold);
  }

  return true;
}

// Nearest neighbor search.
template <int K, typename V>
std::vector<std::pair<typename KdtreeMap<K, V>::VectorKd, V>>
KdtreeMap<K, V>::KnnSearch(const VectorKd& query, size_t k) const {
  std::vector<std::pair<VectorKd, V>> neighbors;

  if (index_ == nullptr) {
    ROS_WARN_THROTTLE(1.0, "KdtreeMap: Cannot search empty index.");
    return neighbors;
  }

  // Convert the input point to the FLANN format.
  flann::Matrix<double> flann_query(new double[K], 1, K);
  for (size_t ii = 0; ii < K; ii++) flann_query[0][ii] = query(ii);

  // Search the kd tree for the nearest neighbor to the query.
  std::vector<std::vector<int>> query_match_indices;
  std::vector<std::vector<double>> query_squared_distances;

  const int num_neighbors_found = index_->knnSearch(
      flann_query, query_match_indices, query_squared_distances,
      static_cast<int>(k), flann::SearchParams(FLANN_CHECKS_UNLIMITED));

  // Assign output.
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[query_match_indices[0][ii]]);

  // Free flann_query memory.
  delete[] flann_query.ptr();

  return neighbors;
}

// Radius search.
template <int K, typename V>
std::vector<std::pair<typename KdtreeMap<K, V>::VectorKd, V>>
KdtreeMap<K, V>::RadiusSearch(const VectorKd& query, double r) const {
  std::vector<std::pair<VectorKd, V>> neighbors;

  if (index_ == nullptr) {
    ROS_WARN("KdtreeMap: cannot search empty index.");
    return neighbors;
  }

  // Convert the input point to the FLANN format.
  flann::Matrix<double> flann_query(new double[K], 1, K);
  for (size_t ii = 0; ii < K; ii++) flann_query[0][ii] = query(ii);

  // Search the kd tree for the nearest neighbor to the query.
  std::vector<std::vector<int>> query_match_indices;
  std::vector<std::vector<double>> query_squared_distances;

  // FLANN checks Euclidean distance squared, so we pass in r * r.
  constexpr double kExactSearch = 0.0;
  constexpr bool kDoNotSort = false;
  const int num_neighbors_found = index_->radiusSearch(
      flann_query, query_match_indices, query_squared_distances, r * r,
      flann::SearchParams(FLANN_CHECKS_UNLIMITED, kExactSearch, kDoNotSort));

  // Assign output.
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[query_match_indices[0][ii]]);

  // Free flann_query memory.
  delete[] flann_query.ptr();

  return neighbors;
}

}  // namespace fastrack

#endif
