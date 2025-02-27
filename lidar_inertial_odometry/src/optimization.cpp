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

Optimization::Optimization(const gtsam::ISAM2Params parameter) : key_(0), lidar_odom_buffer_(3)
{
  smoother_ptr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(5.0, parameter);
  optimizer_ = std::make_shared<gtsam::ISAM2>(parameter);
}

void Optimization::initialize()
{
  pose_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);     // rad,rad,rad,m, m, m
  velocity_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
}

void Optimization::set_initial_value(const Eigen::Matrix4d & initial_pose, const double & timestamp)
{
  key_ = 0;
  gtsam::Pose3 prior_pose(initial_pose);
  gtsam::Vector3 prior_velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias prior_bias;

  gtsam::Values initial_values;
  initial_values.insert(X(key_), prior_pose);
  initial_values.insert(V(key_), prior_velocity);

  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model_));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(key_), prior_velocity, velocity_noise_model_));

  gtsam::FixedLagSmootherKeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values);

  key_++;
}

Eigen::Matrix4d Optimization::update(
  const double & timestamp, const Eigen::Matrix4d & latest_frame,
  const std::deque<sensor_type::Pose> & map_pose_queue, const gtsam::NavState state,
  const gtsam::imuBias::ConstantBias imu_bias)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  gtsam::Pose3 pose_to(latest_frame);
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
  if (lidar_odom_buffer_.empty()) {
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    gtsam::PriorFactor<gtsam::Pose3> lidar_prior_factor(X(key_), pose_to, prior_noise);
    graph.add(lidar_prior_factor);
  } else {
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    gtsam::Pose3 pose_from(lidar_odom_buffer_.back());
    gtsam::BetweenFactor<gtsam::Pose3> lidar_relative_factor(
      X(key_ - 1), X(key_), pose_from.between(pose_to), odom_noise);
    graph.add(lidar_relative_factor);
  }

  if (!map_pose_queue.empty()) {
    const auto map_pose = map_pose_queue.back();
    gtsam::Pose3 map_prior(map_pose.pose);
    gtsam::noiseModel::Diagonal::shared_ptr map_prior_noise =
      gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    gtsam::PriorFactor<gtsam::Pose3> map_prior_factor(X(key_), map_prior, prior_noise6);
    graph.add(map_prior_factor);
    initial_values.insert(X(key_), map_prior);
  } else {
    initial_values.insert(X(key_), pose_to);
  }

  gtsam::PriorFactor<gtsam::Vector3> predict_velocity(
    V(key_), state.v(), gtsam::noiseModel::Isotropic::Precision(3, 1e3));
  graph.add(predict_velocity);
  initial_values.insert(V(key_), state.v());

  gtsam::FixedLagSmootherKeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values);
  smoother_ptr_->update();

  const auto estimated_state = smoother_ptr_->calculateEstimate().at<gtsam::Pose3>(X(key_));
  lidar_odom_buffer_.push_back(estimated_state);
  key_++;

  return estimated_state.matrix().cast<double>();
}
