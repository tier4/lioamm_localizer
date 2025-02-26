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

#ifndef LIDAR_INERTIAL_ODOMETRY__MAP_MANAGER_HPP_
#define LIDAR_INERTIAL_ODOMETRY__MAP_MANAGER_HPP_

#include "lidar_inertial_odometry/submap.hpp"
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

class MapManager
{
public:
  MapManager(const double resolution, const double distance_threshold)
  : resolution_(resolution), distance_threshold_(distance_threshold)
  {
    local_map_.reset(new PointCloud);
  }
  ~MapManager() = default;

  void add_points(const submap::Submap & submap)
  {
    const Eigen::Vector3d keyframe_position = submap.keyframe_pose.block<3, 1>(0, 3);

    for (auto & p : submap.map_points->points) {
      VoxelKey voxel = get_voxel_index(p.getArray3fMap());

      if (voxel_map_.find(voxel) == voxel_map_.end()) {
        voxel_map_[voxel] = Voxel(p, 1000);
      } else {
        voxel_map_[voxel].add_voxel(p);
      }
    }

    // remove voxel
    std::vector<VoxelKey> remove_voxel_lists;
    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
      Eigen::Vector3d voxel_pos = it->second.mean.cast<double>();
      if (distance_threshold_ < (voxel_pos - keyframe_position).norm()) {
        remove_voxel_lists.emplace_back(it->first);
        it = voxel_map_.erase(it);
      } else {
        local_map_->points.emplace_back(it->second.get_point());
        ++it;
      }
    }

    if (!remove_voxel_lists.empty()) {
      remove_map(remove_voxel_lists);
    }
  }

  void remove_map(const std::vector<VoxelKey> & keys)
  {
    local_map_->points.erase(
      std::remove_if(
        local_map_->points.begin(), local_map_->points.end(),
        [&](const PointType & p) {
          VoxelKey key = get_voxel_index(p.getArray3fMap().cast<float>());
          return std::find(keys.begin(), keys.end(), key) != keys.end();
        }),
      local_map_->points.end());

    local_map_->width = local_map_->points.size();
    local_map_->height = 1;
  }

  VoxelKey get_voxel_index(const Eigen::Vector3f & point)
  {
    Eigen::Vector3i voxel_index = (point / resolution_).array().floor().cast<int>();
    return {voxel_index.x(), voxel_index.y(), voxel_index.z()};
  }

  PointCloudPtr get_local_map() { return local_map_; }

private:
  double resolution_;
  double distance_threshold_;

  std::unordered_map<VoxelKey, Voxel> voxel_map_;
  PointCloudPtr local_map_;
};

#endif
