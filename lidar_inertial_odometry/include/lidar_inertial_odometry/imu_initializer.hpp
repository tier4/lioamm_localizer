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

#ifndef LIDAR_INERTIAL_ODOMETRY__IMU_INITIALIZER_HPP_
#define LIDAR_INERTIAL_ODOMETRY__IMU_INITIALIZER_HPP_

#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

class ImuInitializer
{
public:
  ImuInitializer(std::size_t max_queue_size, const double gravity)
  : max_queue_size_(max_queue_size),
    gravity_(gravity),
    initial_orientation_(Eigen::Matrix3d::Identity())
  {
  }
  ~ImuInitializer() {}

  void add_imu(const sensor_type::Imu & imu_data)
  {
    if (initialized_) {
      return;
    }
    imu_buffer_.push_back(imu_data);

    // calculate mean, covariance and gravity orientation
    if (max_queue_size_ < imu_buffer_.size()) {
      // calculate mean of acc and gyro
      auto [acc_mean, acc_cov] =
        lioamm_localizer_utils::compute_mean_and_covariance<Eigen::Vector3d>(
          imu_buffer_, [](const sensor_type::Imu & imu) { return imu.linear_acceleration; });
      auto [gyro_mean, gyro_cov] =
        lioamm_localizer_utils::compute_mean_and_covariance<Eigen::Vector3d>(
          imu_buffer_, [](const sensor_type::Imu & imu) { return imu.angular_velocity; });

      // z-axis
      const Eigen::Vector3d & z_axis = acc_mean.normalized();
      // x-axis
      Eigen::Vector3d x_axis =
        Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
      x_axis.normalize();
      // y-axis
      Eigen::Vector3d y_axis = z_axis.cross(x_axis);
      y_axis.normalize();

      // initial orientation
      Eigen::Matrix3d initial_R;
      initial_R.block<3, 1>(0, 0) = x_axis;
      initial_R.block<3, 1>(0, 1) = y_axis;
      initial_R.block<3, 1>(0, 2) = z_axis;
      initial_orientation_ =
        Eigen::Quaterniond(initial_R).normalized().toRotationMatrix().transpose();

      // estimate initial imu gravity orientation
      gravity_vector_ = -acc_mean / acc_mean.norm() * gravity_;

      imu_scale_ = gravity_ / acc_mean.norm();

      auto [acc_mean_with_gravity, acc_cov_with_gravity] =
        lioamm_localizer_utils::compute_mean_and_covariance<Eigen::Vector3d>(
          imu_buffer_, [this](const sensor_type::Imu & imu) {
            return imu.linear_acceleration + gravity_vector_;
          });

      acc_mean_ = acc_mean_with_gravity;
      acc_cov_ = acc_cov_with_gravity;
      gyro_mean_ = gyro_mean;
      gyro_cov_ = gyro_cov;

      initialized_ = true;
    }
  }
  bool is_initialized() { return initialized_; }

  Eigen::Vector3d get_acc_mean() { return acc_mean_; }
  Eigen::Vector3d get_gyro_mean() { return gyro_mean_; }
  Eigen::Vector3d get_acc_cov() { return acc_cov_; }
  Eigen::Vector3d get_gyro_cov() { return gyro_cov_; }
  Eigen::Vector3d get_gravity() { return gravity_vector_; }
  Eigen::Matrix3d get_initial_orientation() { return initial_orientation_; }
  double get_imu_scale() { return imu_scale_; }

  void clear()
  {
    imu_buffer_.clear();
    acc_mean_.setZero();
    gyro_mean_.setZero();
    acc_cov_.setZero();
    gyro_cov_.setZero();
    initialized_ = false;
  }

private:
  std::deque<sensor_type::Imu> imu_buffer_;
  bool initialized_{false};
  std::size_t max_queue_size_;
  double gravity_;

  Eigen::Vector3d acc_mean_;
  Eigen::Vector3d gyro_mean_;
  Eigen::Vector3d acc_cov_;
  Eigen::Vector3d gyro_cov_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Matrix3d initial_orientation_;
  double imu_scale_;
};

#endif
