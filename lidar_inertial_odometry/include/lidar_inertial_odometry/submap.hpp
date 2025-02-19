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

#ifndef LIDAR_INERTIAL_ODOMETRY__SUBMAP_HPP_
#define LIDAR_INERTIAL_ODOMETRY__SUBMAP_HPP_

#include "lioamm_localizer_common/point_type.hpp"

#include <pcl_ros/transforms.hpp>

namespace submap
{

struct Submap
{
  PointCloudPtr raw_points;
  PointCloudPtr map_points;
  Eigen::Matrix4d keyframe_pose;
  Submap()
  {
    keyframe_pose = Eigen::Matrix4d::Identity();
    raw_points.reset(new PointCloud);
    map_points.reset(new PointCloud);
  }
  Submap(const Eigen::Matrix4d & keyframe_pose, const PointCloudPtr & raw_points)
  {
    this->map_points.reset(new PointCloud);
    this->keyframe_pose = keyframe_pose;
    this->raw_points = raw_points;
    pcl::transformPointCloud(*this->raw_points, *this->map_points, this->keyframe_pose);
  }
};

}  // namespace submap

#endif
