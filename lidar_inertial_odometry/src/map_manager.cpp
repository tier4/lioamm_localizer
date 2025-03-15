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

#include "lidar_inertial_odometry/map_mananger.hpp"

MapManager::MapManager(
  const double resolution, const int max_submap_size, const double translation_threshold,
  const double rotation_threshold)
: resolution_(resolution),
  max_submap_size_(max_submap_size),
  translation_threshold_(translation_threshold),
  rotation_threshold_(rotation_threshold)
{
  local_map_.reset(new PointCloud);
  keyframe_points_.reset(new PointCloud);

  voxel_grid_.setLeafSize(resolution, resolution, resolution);

  // TODO: multi thread
  thread_ = std::thread(&MapManager::task_runner, this);
}

MapManager::~MapManager()
{
  {
    std::lock_guard<std::mutex> lock(task_queue_mutex_);
    stop_ = true;
  }

  task_queue_condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void MapManager::task_runner()
{
  while (!stop_) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(task_queue_mutex_);
      task_queue_condition_.wait(lock, [this] { return !task_queue_.empty() || stop_; });

      task = std::move(task_queue_.front());
      task_queue_.pop_front();
    }
    task();
  }
}

std::future<PointCloudPtr> MapManager::add_map_points(
  const sensor_type::Lidar & sensor_measurement, const Eigen::Matrix4d & keyframe_pose)
{
  auto function = std::bind(&MapManager::build_map_task, this, sensor_measurement, keyframe_pose);

  auto task =
    std::make_shared<std::packaged_task<PointCloudPtr()>>([function]() { return function(); });

  auto future = task->get_future();

  add_task_queue([task]() { (*task)(); });

  return future;
}

template <typename F>
void MapManager::add_task_queue(F && task)
{
  {
    std::lock_guard<std::mutex> lock(task_queue_mutex_);
    task_queue_.push_back(std::forward<F>(task));
  }
  task_queue_condition_.notify_one();
}

PointCloudPtr MapManager::build_map_task(
  const sensor_type::Lidar & sensor_measurement, const Eigen::Matrix4d & keyframe_pose)
{
  PointCloudPtr downsample_cloud(new PointCloud);
  voxel_grid_.setInputCloud(sensor_measurement.raw_points);
  voxel_grid_.filter(*downsample_cloud);
  submap::Submap submap(keyframe_pose, downsample_cloud);
  submaps_.push_back(submap);

  PointType query_point;
  query_point.getVector3fMap() = submap.keyframe_pose.block<3, 1>(0, 3).cast<float>();
  keyframe_points_->points.emplace_back(query_point);
  kd_tree_.setInputCloud(keyframe_points_);

  int submap_size = submaps_.size();
  PointCloudPtr local_map(new PointCloud);
  for (int idx = 0; idx < max_submap_size_; idx++) {
    if ((submap_size - 1 - idx) < 0) continue;
    *local_map += *submaps_[submap_size - 1 - idx].map_points;
  }
  voxel_grid_.setInputCloud(local_map);
  voxel_grid_.filter(*local_map);

  {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    local_map_.reset(new PointCloud);
    local_map_ = local_map;
    new_map_is_ready_ = true;
  }

  return local_map_;
}

bool MapManager::is_map_update(const Eigen::Matrix4d & pose)
{
  bool is_map_update = false;

  if (submaps_.empty()) {
    is_map_update = true;
  } else {
    PointType query_point;
    query_point.getVector3fMap() = pose.block<3, 1>(0, 3).cast<float>();

    const Eigen::Vector3d candidate_pos = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d candidate_rot = pose.block<3, 3>(0, 0);

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kd_tree_.nearestKSearch(query_point, 1, indices, distances)) {
      Eigen::Vector3d closest_pos = submaps_[indices[0]].keyframe_pose.block<3, 1>(0, 3);
      Eigen::Matrix3d closest_rot = submaps_[indices[0]].keyframe_pose.block<3, 3>(0, 0);

      const double delta_p = (candidate_pos - closest_pos).norm();
      const double delta_rot = Eigen::AngleAxisd(closest_rot.transpose() * candidate_rot).angle();

      if (translation_threshold_ < delta_p || rotation_threshold_ < delta_rot) {
        is_map_update = true;
      }
    }
  }

  return is_map_update;
}
