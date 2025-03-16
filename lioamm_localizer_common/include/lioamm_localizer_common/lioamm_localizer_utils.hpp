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

#include <deque>
#include <numeric>

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
  // clang-format off
  matrix <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  // clang-format on
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

inline Eigen::Vector3d convert_matrix_to_euler(const Eigen::Matrix3d & matrix)
{
  Eigen::AngleAxisd angle_axis(matrix);
  return angle_axis.angle() * angle_axis.axis();
}

inline Eigen::Quaterniond convert_euler_to_quaternion(const Eigen::Vector3d & euler)
{
  Eigen::AngleAxisd angle_axis(euler.norm(), euler.normalized());
  return Eigen::Quaterniond(angle_axis);
}

inline Eigen::Vector3d convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  Eigen::Quaterniond quaternion_eigen(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

  return convert_matrix_to_euler(quaternion_eigen.toRotationMatrix());
}

inline Eigen::Matrix3d convert_euler_to_rotation_matrix(const Eigen::Vector3d & euler)
{
  return convert_euler_to_quaternion(euler).toRotationMatrix();
}

template <typename T, typename D, typename E>
inline T compute_mean(const D & data, E extractor)
{
  T sum = std::accumulate(
    data.begin(), data.end(), T::Zero().eval(),
    [&](const T & data, const typename D::value_type & element) -> T {
      return data + extractor(element);
    });

  return sum / data.size();
}

template <typename T, typename D, typename E>
std::tuple<T, T> compute_mean_and_covariance(const D & data, E extractor)
{
  const size_t size = data.size();

  T mean = std::accumulate(
    data.begin(), data.end(), T::Zero().eval(),
    [&extractor](const T & sum, const auto & data) -> T { return sum + extractor(data); });
  mean /= size;

  T covariance = std::accumulate(
    data.begin(), data.end(), T::Zero().eval(),
    [&mean, &extractor](const T & sum, const auto & data) -> T {
      return sum + (extractor(data) - mean).cwiseAbs2().eval();
    });
  covariance /= (size - 1);

  return std::make_tuple(mean, covariance);
}

inline std::vector<double> normalize(const std::vector<double> & times)
{
  if (times.empty()) return {};

  double min_element = *std::min_element(times.begin(), times.end());
  double max_element = *std::max_element(times.begin(), times.end());

  if (min_element == max_element) {
    return std::vector<double>(times.size(), 0.0);
  }

  std::vector<double> normalized;
  normalized.reserve(times.size());

  for (double t : times) {
    normalized.push_back((t - min_element) / (max_element - min_element));
  }

  return normalized;
}

}  // namespace lioamm_localizer_utils

#endif
