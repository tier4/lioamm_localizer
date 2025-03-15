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

Optimization::Optimization(const gtsam::ISAM2Params parameter) : lidar_odom_buffer_(3), key_(0)
{
  smoother_ptr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(5.0, parameter);
}

void Optimization::set_initial_value(const double & timestamp, const Eigen::Matrix4d & initial_pose)
{
  key_ = 0;
  gtsam::Pose3 prior_pose(initial_pose);
  lidar_odom_buffer_.push_back(prior_pose);
  gtsam::Vector3 prior_velocity = gtsam::Vector3::Zero();

  const auto pose_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
  const auto velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), prior_pose);
  initial_values.insert(V(key_), prior_velocity);

  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(key_), prior_velocity, velocity_noise_model));

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values);
  smoother_ptr_->update();

  latest_state_ = gtsam::NavState(prior_pose, prior_velocity);
}

Eigen::Matrix4d Optimization::update(
  const double & timestamp, const Eigen::Matrix4d & lidar_pose_matrix,
  const gtsam::NavState & predict_state)
{
  key_++;
  gtsam::NonlinearFactorGraph graph;

  gtsam::Pose3 pose_to(lidar_pose_matrix);

  gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(
    (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::Pose3 pose_from(lidar_odom_buffer_.back());
  gtsam::BetweenFactor<gtsam::Pose3> lidar_relative_factor(
    X(key_ - 1), X(key_), pose_from.between(pose_to), odom_noise);
  graph.add(lidar_relative_factor);

  gtsam::PriorFactor<gtsam::Vector3> predict_velocity(
    V(key_), predict_state.v(), gtsam::noiseModel::Isotropic::Precision(3, 1e3));
  graph.add(predict_velocity);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), pose_to);
  initial_values.insert(V(key_), predict_state.v());

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  new_timestamp[V(key_)] = timestamp;

  smoother_ptr_->update(graph, initial_values);
  smoother_ptr_->update();

  const auto result = smoother_ptr_->calculateEstimate();
  latest_state_ =
    gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), result.at<gtsam::Vector3>(V(key_)));
  lidar_odom_buffer_.push_back(latest_state_.pose());

  return latest_state_.pose().matrix().cast<double>();
}
