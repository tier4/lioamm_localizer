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

#ifndef MAP_MATCHER__SENSOR_TYPE_HPP_
#define MAP_MATCHER__SENSOR_TYPE_HPP_

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

struct PointCloud
{
  double stamp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessing_points;
  PointCloud()
  {
    raw_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    preprocessing_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
};

struct Odometry
{
  double stamp;
  Eigen::Matrix4f odometry;
};

struct Measurement
{
  sensor_type::PointCloud lidar_points;
  std::deque<sensor_type::Imu> imu_queue;
};

}  // namespace sensor_type

#endif
