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
  const double resolution, const double translation_threshold, const double rotation_threshold)
: resolution_(resolution),
  translation_threshold_(translation_threshold),
  rotation_threshold_(rotation_threshold)
{
  local_map_.reset(new PointCloud);
  keyframe_points_.reset(new PointCloud);

  voxel_grid_.setLeafSize(resolution, resolution, resolution);

  map_thread_ = std::thread(&MapManager::process_queue, this);
}

MapManager::~MapManager()
{
  {
    std::lock_guard<std::mutex> lock(task_queue_mutex_);
    stop_ = true;
  }

  task_queue_condition_.notify_all();
  if (map_thread_.joinable()) {
    map_thread_.join();
  }
}

void MapManager::process_queue()
{
  while (!stop_) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(task_queue_mutex_);
      task_queue_condition_.wait(lock, [this] { return !task_queue_.empty() || stop_; });

      task = task_queue_.front();
      task_queue_.pop_front();
    }
    task();
  }
}

void MapManager::add_map_points(
  const sensor_type::Lidar & sensor_measurement, const Eigen::Matrix4d & keyframe_pose)
{
  submap::Submap submap(keyframe_pose, sensor_measurement.raw_points);
  auto task = std::bind(&MapManager::build_map_task, this, submap);
  {
    std::lock_guard<std::mutex> lock(task_queue_mutex_);
    task_queue_.emplace_back(task);
  }
  task_queue_condition_.notify_one();
}

void MapManager::build_map_task(const submap::Submap & submap)
{
  submaps_.emplace_back(submap);

  PointType query_point;
  query_point.getVector3fMap() = submap.keyframe_pose.block<3, 1>(0, 3).cast<float>();
  keyframe_points_->points.emplace_back(query_point);
  kd_tree_.setInputCloud(keyframe_points_);

  for (const auto & point : submap.map_points->points) {
    auto key = get_voxel_index(point.getVector3fMap());
    if (voxel_map_.find(key) == voxel_map_.end()) {
      voxel_map_[key] = Voxel(point, 1000);
    } else {
      voxel_map_[key].add_voxel(point);
    }
  }

  if (20 < submaps_.size()) {
    auto map_points = submaps_.front();
    for (const auto & point : map_points.map_points->points) {
      auto key = get_voxel_index(point.getVector3fMap());
      if (voxel_map_.count(key)) {
        voxel_map_.erase(key);
      }
    }
    submaps_.pop_front();
  }

  PointCloudPtr updated_cloud(new PointCloud);
  for (const auto & [key, points] : voxel_map_) {
    auto mean = points.mean;
    PointType p;
    p.x = mean.x();
    p.y = mean.y();
    p.z = mean.z();
    updated_cloud->points.emplace_back(p);
  }

  {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    local_map_ = updated_cloud;
    new_map_is_ready_ = true;
  }
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
