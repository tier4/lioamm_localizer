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
#include "lioamm_localizer_common/concurrent_queue.hpp"
#include "lioamm_localizer_common/hash_voxel_map.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <condition_variable>
#include <future>
#include <queue>
#include <thread>

class MapManager
{
  using mapping_task = std::packaged_task<PointCloudPtr(submap::Submap)>;

public:
  MapManager(
    const double resolution, const int max_submap_size, const double translation_threshold,
    const double rotation_threshold);
  ~MapManager();

  void task_runner();

  template <typename F>
  void add_task_queue(F && task);

  std::future<PointCloudPtr> add_map_points(
    const sensor_type::Lidar & sensor_measurement, const Eigen::Matrix4d & keyframe_pose);
  PointCloudPtr build_map_task(
    const sensor_type::Lidar & sensor_measurement, const Eigen::Matrix4d & keyframe_pose);

  bool is_map_update(const Eigen::Matrix4d & pose);

  inline bool has_map_changed()
  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return new_map_is_ready_;
  }

  inline void reset()
  {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    new_map_is_ready_ = false;
  }

  inline PointCloudPtr get_local_map()
  {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    return local_map_;
  }

  VoxelKey get_voxel_index(const Eigen::Vector3f & point)
  {
    Eigen::Vector3i voxel_index = (point / resolution_).array().floor().cast<int>();
    return {voxel_index.x(), voxel_index.y(), voxel_index.z()};
  }

private:
  std::thread thread_;

  std::mutex task_queue_mutex_;
  std::shared_mutex map_mutex_;
  std::condition_variable task_queue_condition_;
  std::deque<std::function<void()>> task_queue_{};

  PointCloudPtr local_map_;
  PointCloudPtr keyframe_points_;
  ConcurrentQueue<submap::Submap> submaps_;

  std::atomic<bool> new_map_is_ready_{false};
  bool stop_{false};

  double resolution_;
  pcl::VoxelGrid<PointType> voxel_grid_;
  pcl::KdTreeFLANN<PointType> kd_tree_;

  std::unordered_map<VoxelKey, Voxel> voxel_map_;

  int max_submap_size_;
  double translation_threshold_;
  double rotation_threshold_;
};

#endif
