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

#ifndef LIOAMM_LOCALIZER_COMMON__LIOAMM_LOCALIZER_UTILS_HPP_
#define LIOAMM_LOCALIZER_COMMON__LIOAMM_LOCALIZER_UTILS_HPP_

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/convert.h>

namespace lioamm_localizer_utils
{

inline Eigen::Matrix4f geometry_pose_to_matrix(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  return matrix;
}

inline geometry_msgs::msg::Pose convert_matrix_to_pose(const Eigen::Matrix4f matrix)
{
  geometry_msgs::msg::Pose pose;

  const Eigen::Vector3d position = matrix.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0).cast<double>());

  pose.position = tf2::toMsg(position);
  pose.orientation = tf2::toMsg(quaternion);

  return pose;
}

inline Eigen::Matrix4d get_matrix(
  const Eigen::Vector3d translation, const Eigen::Quaterniond quaternion)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = translation;

  return matrix;
}

inline geometry_msgs::msg::Pose convert_transform_to_pose(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform_stamped.transform.translation.x;
  pose.position.y = transform_stamped.transform.translation.y;
  pose.position.z = transform_stamped.transform.translation.z;
  pose.orientation.w = transform_stamped.transform.rotation.w;
  pose.orientation.x = transform_stamped.transform.rotation.x;
  pose.orientation.y = transform_stamped.transform.rotation.y;
  pose.orientation.z = transform_stamped.transform.rotation.z;
  return pose;
}

inline Eigen::Matrix4f convert_pose_to_matrix(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  return matrix;
}

inline Eigen::Matrix4f convert_transform_to_matrix(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  return convert_pose_to_matrix(convert_transform_to_pose(transform_stamped));
}

inline Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix;
  return matrix;
}

inline Eigen::Transform<double, 3, Eigen::Affine> get_eigen_transform(
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  Eigen::Quaterniond quaternion(
    transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
    transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
  Eigen::Transform<double, 3, Eigen::Affine> transform(quaternion);

  return transform;
}

}  // namespace lioamm_localizer_utils

#endif

