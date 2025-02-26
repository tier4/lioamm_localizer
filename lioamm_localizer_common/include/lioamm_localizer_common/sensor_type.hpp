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

#ifndef LIOAMM_LOCALIZER_COMMON__SENSOR_TYPE_HPP_
#define LIOAMM_LOCALIZER_COMMON__SENSOR_TYPE_HPP_

#include "lioamm_localizer_common/point_type.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>

namespace sensor_type
{

struct Imu
{
  double stamp;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

struct Lidar
{
  double stamp;
  double lidar_start_time;
  double lidar_end_time;
  std::vector<double> timestamp;
  PointCloudPtr raw_points;
  PointCloudPtr preprocessing_points;
  Lidar()
  {
    raw_points.reset(new PointCloud);
    preprocessing_points.reset(new PointCloud);
  }
};

struct Pose
{
  double stamp;
  Eigen::Matrix4d pose;
};

struct Measurement
{
  sensor_type::Lidar lidar_points;
  std::deque<sensor_type::Imu> imu_queue;
  std::deque<sensor_type::Pose> map_pose_queue;
};

}  // namespace sensor_type

#endif
