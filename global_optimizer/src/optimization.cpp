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

#include "global_optimizer/optimization.hpp"

#include <cmath>

Optimization::Optimization(const gtsam::ISAM2Params parameter)
: lidar_odom_buffer_(3), parameter_(parameter), key_(0)
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

  gtsam::Values initial_values;
  initial_values.insert(X(key_), prior_pose);

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model));

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;

  smoother_ptr_->update(graph_, initial_values);
  smoother_ptr_->update();

  lidar_odom_buffer_.push_back(prior_pose);

  graph_.resize(0);

  key_++;

  latest_state_ = gtsam::NavState(prior_pose, prior_velocity);
}

void Optimization::add_map_matching_factor(const sensor_type::Pose & map_matching_pose)
{
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.25, 0.25, 0.25).finished());
  // (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
  // (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e2);

  gtsam::Pose3 map_prior_pose(map_matching_pose.pose);
  gtsam::PriorFactor<gtsam::Pose3> map_pose_prior_factor(X(key_), map_prior_pose, prior_noise6);
  graph_.add(map_pose_prior_factor);
}

void Optimization::add_odom_factor(
  const sensor_type::Pose & odom_pose, const bool & map_matching_is_fail)
{
  gtsam::Pose3 pose_to(odom_pose.pose);
  if (lidar_odom_buffer_.empty()) {
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3).finished());
    gtsam::PriorFactor<gtsam::Pose3> odom_prior_factor(X(key_), pose_to, prior_noise);
    graph_.add(odom_prior_factor);
  } else {
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
    gtsam::noiseModel::Diagonal::shared_ptr between_noise;

    if (map_matching_is_fail) {
      between_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished() * 0.1);
    } else {
      between_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    }

    gtsam::Pose3 pose_from(lidar_odom_buffer_.back());
    gtsam::BetweenFactor<gtsam::Pose3> odom_between_factor(
      X(key_ - 1), X(key_), pose_from.between(pose_to), between_noise);
    graph_.add(odom_between_factor);
  }
  lidar_odom_buffer_.push_back(pose_to);
}

Eigen::Matrix4d Optimization::update(const Eigen::Matrix4d & initial_pose)
{
  gtsam::Pose3 initial_prior_pose(initial_pose);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), initial_prior_pose);

  smoother_ptr_->update(graph_, initial_values);
  smoother_ptr_->update();

  const auto result = smoother_ptr_->calculateEstimate();

  covariance_ = smoother_ptr_->marginalCovariance(gtsam::Symbol('x', key_));

  latest_state_ = gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), gtsam::Vector3::Zero());

  graph_.resize(0);
  key_++;

  return latest_state_.pose().matrix().cast<double>();
}
