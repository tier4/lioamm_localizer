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

#include "lidar_inertial_odometry/optimization.hpp"

#include <cmath>

Optimization::Optimization(const gtsam::ISAM2Params parameter) : key_(0)
{
  smoother_ptr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(5.0, parameter);
  optimizer_ = std::make_shared<gtsam::ISAM2>(parameter);

  const double imu_acc_bias = 1.2123362494392119e-04;
  const double imu_gyro_bias = 8.6572985145653080e-05;
  imu_bias_noise_between_ = (gtsam::Vector(6) << imu_acc_bias, imu_acc_bias, imu_acc_bias,
                             imu_gyro_bias, imu_gyro_bias, imu_gyro_bias)
                              .finished();
}

void Optimization::set_initial_value(
  const Eigen::Matrix4d & initial_pose, const Eigen::VectorXd & imu_bias, const double & timestamp)
{
  key_ = 0;
  gtsam::imuBias::ConstantBias prior_imu_bias(imu_bias.head<3>(), imu_bias.tail<3>());
  gtsam::Pose3 prior_pose(initial_pose);
  gtsam::Vector3 prior_velocity = gtsam::Vector3::Zero();

  const auto pose_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
  const auto velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-6);
  const auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-5);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), prior_pose);
  initial_values.insert(V(key_), prior_velocity);
  initial_values.insert(B(key_), prior_imu_bias);

  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(key_), prior_velocity, velocity_noise_model));
  graph.add(
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), prior_imu_bias, bias_noise_model));

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;
  new_timestamp[B(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values, new_timestamp);

  latest_state_ = gtsam::NavState(prior_pose, prior_velocity);
  latest_imu_bias_ = prior_imu_bias;
}

Eigen::Matrix4d Optimization::update(
  const double & timestamp, const Eigen::Matrix4d & lidar_pose_matrix,
  const gtsam::PreintegratedImuMeasurements & imu_integration_ptr)
{
  key_++;
  gtsam::NonlinearFactorGraph graph;

  const auto imu_integration =
    dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(imu_integration_ptr);

  // IMU factor
  gtsam::ImuFactor imu_factor(
    X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_), imu_integration_ptr);
  // IMU bias factor
  gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> imu_bias_factor(
    B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
    gtsam::noiseModel::Diagonal::Sigmas(
      std::sqrt(imu_integration_ptr.deltaTij()) * imu_bias_noise_between_));

  // add IMU factor
  graph.add(imu_factor);
  graph.add(imu_bias_factor);

  gtsam::Pose3 lidar_pose(lidar_pose_matrix);

  gtsam::PriorFactor<gtsam::Pose3> lidar_prior_factor(
    X(key_), lidar_pose, gtsam::noiseModel::Isotropic::Precision(6, 1e6));
  graph.add(lidar_prior_factor);

  auto predict_state = imu_integration_ptr.predict(latest_state_, latest_imu_bias_);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), lidar_pose);
  initial_values.insert(V(key_), predict_state.v());
  initial_values.insert(B(key_), latest_imu_bias_);

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;
  new_timestamp[B(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values, new_timestamp);
  smoother_ptr_->update();

  const auto result = smoother_ptr_->calculateEstimate();
  latest_state_ =
    gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), result.at<gtsam::Vector3>(V(key_)));
  latest_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));

  return latest_state_.pose().matrix().cast<double>();
}
