// Copyright 2024 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIOAMM_LOCALIZER_COMMON__HASH_VOXEL_MAP_HPP_
#define LIOAMM_LOCALIZER_COMMON__HASH_VOXEL_MAP_HPP_

#include "lioamm_localizer_common/point_type.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <execution>

struct Voxel
{
  int max_points_queue_size;
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();
  std::vector<Eigen::Vector3f> points;

  Voxel() : max_points_queue_size(0) {}
  Voxel(const PointType & point, const int max_points_queue_size)
  : max_points_queue_size(max_points_queue_size)
  {
    points.reserve(max_points_queue_size);
    points.emplace_back(point.getVector3fMap());
    mean = point.getVector3fMap();
  }

  void add_voxel(const PointType & point)
  {
    if (points.size() < max_points_queue_size) {
      const int num_points = points.size();
      const Eigen::Vector3f point_vec = point.getVector3fMap();
      mean = (num_points * mean + point_vec) / (num_points + 1);
      points.emplace_back(point_vec);
    }
  }

  PointType get_point() const
  {
    PointType point;
    point.getVector3fMap() = mean;
    return point;
  }
};

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & voxel) const
  {
    return x == voxel.x && y == voxel.y && z == voxel.z;
  }
};

// Hash function
namespace std
{
template <>
struct hash<VoxelKey>
{
  size_t operator()(const VoxelKey & k) const
  {
    return ((size_t)k.x * 73856093) ^ ((size_t)k.y * 19349663) ^ ((size_t)k.z * 83492791);
  }
};
}  // namespace std

#endif
