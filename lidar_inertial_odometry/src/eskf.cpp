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

#include "lidar_inertial_odometry/eskf.hpp"

namespace eskf
{

ESKF::ESKF(ESKFConfig config) : config_(config)
{
  Fx_ = Eigen::Matrix<double, 18, 18>::Identity();

  Fi_ = Eigen::Matrix<double, 18, 12>::Zero();
  Fi_.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Qi_ = Eigen::Matrix<double, 12, 12>::Identity();

  Qi_.block<3, 3>(0, 0) = std::pow(config_.acc_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(3, 3) = std::pow(config_.gyro_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(6, 6) = std::pow(config_.acc_bias_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(9, 9) = std::pow(config_.gyro_bias_noise, 2) * Eigen::Matrix3d::Identity();

  V_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> noise;
  noise << config_.translation_noise, config_.translation_noise, config_.translation_noise,
    config_.rotation_noise, config_.rotation_noise, config_.rotation_noise;
  V_ = noise.asDiagonal();

  G_ = Eigen::Matrix<double, 18, 18>::Identity();

  error_.setZero();
}

ESKF::~ESKF()
{
}

void ESKF::set_Q(const Eigen::Vector3d & acc_cov, const Eigen::Vector3d & gyro_cov)
{
  Qi_.block<3, 3>(0, 0).diagonal() = acc_cov;
  Qi_.block<3, 3>(3, 3).diagonal() = gyro_cov;
  Qi_.block<3, 3>(6, 6) *= std::pow(config_.acc_bias_noise, 2);
  Qi_.block<3, 3>(9, 9) *= std::pow(config_.gyro_bias_noise, 2);
}

void ESKF::initialize(
  const Eigen::Matrix4d initial_pose, const Eigen::Vector<double, 6> bias,
  const Eigen::Vector3d gravity, const double stamp)
{
  State initial_state;
  initial_state.stamp = stamp;
  initial_state.position = initial_pose.block<3, 1>(0, 3);
  initial_state.velocity = Eigen::Vector3d::Zero();
  initial_state.quaternion = Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0));
  initial_state.acc_bias = bias.head<3>();
  initial_state.gyro_bias = bias.tail<3>();
  initial_state.gravity = gravity;
  initial_state.quaternion.normalize();
  state_ = initial_state;
}

void ESKF::predict(const sensor_type::Imu imu_measurement)
{
  const double dt = imu_measurement.stamp - state_.stamp;
  if (dt < 0.0) {
    return;
  }

  const Eigen::Matrix3d R = state_.quaternion.toRotationMatrix();
  const Eigen::Vector3d acc = imu_measurement.linear_acceleration - state_.acc_bias;
  const Eigen::Vector3d gyro = imu_measurement.angular_velocity - state_.gyro_bias;

  state_.position =
    state_.position + state_.velocity * dt + 0.5 * (R * acc + state_.gravity) * dt * dt;
  state_.velocity = state_.velocity + (R * acc + state_.gravity) * dt;
  state_.quaternion = Eigen::Quaterniond(R * (Sophus::SO3d::exp(gyro * dt).matrix()));
  state_.quaternion.normalize();

  Fx_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(3, 6) = -R * lioamm_localizer_utils::skew_symmetric_matrix(acc) * dt;
  Fx_.block<3, 3>(3, 9) = -R * dt;
  Fx_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(6, 6) = Sophus::SO3d::exp((gyro * dt)).matrix().transpose();
  Fx_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  state_.P = Fx_ * state_.P * Fx_.transpose() + Fi_ * Qi_ * Fi_.transpose();
  state_.stamp = imu_measurement.stamp;
}

Eigen::Matrix4d ESKF::update(sensor_type::Pose & pose_measurement)
{
  // calculate measurement error
  Eigen::Vector<double, 6> residual = Eigen::Vector<double, 6>::Zero(6);
  residual.head<3>() = pose_measurement.pose.block<3, 1>(0, 3) - state_.position;
  residual.tail<3>() = lioamm_localizer_utils::convert_matrix_to_euler(
    state_.quaternion.toRotationMatrix().transpose() * pose_measurement.pose.block<3, 3>(0, 0));

  // calculation jacobian H
  // TODO: update hessian
  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 18, 6> K =
    state_.P * H.transpose() * (H * state_.P * H.transpose() + V_).inverse();
  error_ = K * residual;
  state_.P = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * state_.P;

  state_.position += error_.block<3, 1>(0, 0);
  state_.velocity += error_.block<3, 1>(3, 0);
  state_.quaternion *= Eigen::Quaterniond(Sophus::SO3d::exp(error_.block<3, 1>(6, 0)).matrix());
  state_.quaternion.normalize();
  state_.acc_bias += error_.block<3, 1>(9, 0);
  state_.gyro_bias += error_.block<3, 1>(12, 0);
  state_.gravity += error_.block<3, 1>(15, 0);

  // reset ESKF
  G_.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() -
    0.5 * lioamm_localizer_utils::skew_symmetric_matrix(error_.block<3, 1>(6, 0));
  state_.P = G_ * state_.P * G_.transpose();
  error_.setZero();

  return lioamm_localizer_utils::get_matrix(state_.position, state_.quaternion);
}

}  // namespace eskf
