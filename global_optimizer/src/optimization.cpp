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
  const auto velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);

  gtsam::Values initial_values;
  initial_values.insert(X(key_), prior_pose);
  // initial_values.insert(V(key_), prior_velocity);

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model));
  // graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(key_), prior_velocity, velocity_noise_model));

  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamp;
  new_timestamp[X(key_)] = timestamp;
  // new_timestamp[V(key_)] = timestamp;

  smoother_ptr_->update(graph_, initial_values);
  smoother_ptr_->update();

  lidar_odom_buffer_.push_back(prior_pose);

  graph_.resize(0);

  key_++;

  latest_state_ = gtsam::NavState(prior_pose, prior_velocity);
}

void Optimization::add_map_matching_factor(
  const sensor_type::Pose & map_matching_pose, const double & score)
{
  // スコアに基づく動的なノイズモデルの作成
  double base_var_pos = 0.01;  // 位置の基本分散
  double base_var_rot = 0.25;  // 回転の基本分散

  // スコア履歴の更新
  score_history_.push_back(score);
  if (score_history_.size() > 10) {  // 履歴は過去10フレーム分
    score_history_.pop_front();
  }

  // スコア回復の検出
  bool is_recovering = score_check_recovery(score);

  if (is_recovering && !recovery_mode_) {
    std::cout << "check recovery" << std::endl;
    // 回復モード開始
    recovery_mode_ = true;
    recovery_frames_ = 0;
    last_odom_pose_ = lidar_odom_buffer_.back();
    recovery_target_pose_ = gtsam::Pose3(map_matching_pose.pose);
  }

  gtsam::noiseModel::Diagonal::shared_ptr odom_noise;

  if (recovery_mode_) {
    // 回復モード中は徐々に重みを上げる
    double progress = static_cast<double>(recovery_frames_) / max_recovery_frames_;
    double adaptive_factor = 10.0 * (1.0 - progress) + 1.0;  // 10.0から1.0へ徐々に減少
    std::cout << adaptive_factor << std::endl;

    odom_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << base_var_pos * adaptive_factor, base_var_pos * adaptive_factor,
       base_var_pos * adaptive_factor, base_var_rot * adaptive_factor,
       base_var_rot * adaptive_factor, base_var_rot * adaptive_factor)
        .finished());

    recovery_frames_++;
    if (recovery_frames_ >= max_recovery_frames_) {
      recovery_mode_ = false;
    }
  } else {
    // 通常モード
    odom_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << base_var_pos,
                                                         base_var_pos, base_var_pos, base_var_rot,
                                                         base_var_rot, base_var_rot)
                                                          .finished());
  }

  gtsam::Pose3 map_prior_pose(map_matching_pose.pose);
  gtsam::PriorFactor<gtsam::Pose3> map_pose_prior_factor(X(key_), map_prior_pose, odom_noise);
  graph_.add(map_pose_prior_factor);
}

void Optimization::add_map_matching_factor(const sensor_type::Pose & map_matching_pose)
{
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.25, 0.25, 0.25).finished());
  // (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
  // (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

  gtsam::Pose3 map_prior_pose(map_matching_pose.pose);
  gtsam::PriorFactor<gtsam::Pose3> map_pose_prior_factor(X(key_), map_prior_pose, odom_noise);
  graph_.add(map_pose_prior_factor);
}

Eigen::Matrix4d Optimization::add_odom_factor(
  const sensor_type::Pose & odom_pose, const bool & map_matching_is_fail)
{
  // Eigen::Matrix4d odom_diff(Eigen::Matrix4d::Identity());

  // gtsam::Pose3 pose_to(odom_pose.pose);
  // if (lidar_odom_buffer_.empty()) {
  //   gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(
  //     (gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3).finished());
  //   gtsam::PriorFactor<gtsam::Pose3> odom_prior_factor(X(key_), pose_to, prior_noise);
  //   graph_.add(odom_prior_factor);
  // } else {
  //   const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
  //   gtsam::noiseModel::Diagonal::shared_ptr between_noise;

  //   between_noise = gtsam::noiseModel::Diagonal::Variances(
  //     // (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
  //     (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  //   gtsam::Pose3 pose_from(lidar_odom_buffer_.back());
  //   gtsam::BetweenFactor<gtsam::Pose3> odom_between_factor(
  //     X(key_ - 1), X(key_), pose_from.between(pose_to), prior_noise6);
  //   graph_.add(odom_between_factor);

  //   odom_diff = (pose_from.between(pose_to).matrix());
  // }
  // lidar_odom_buffer_.push_back(pose_to);

  // return odom_diff;

  Eigen::Matrix4d odom_diff(Eigen::Matrix4d::Identity());

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
        // (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    } else {
      between_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
      // (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    }

    gtsam::Pose3 pose_from(lidar_odom_buffer_.back());
    gtsam::BetweenFactor<gtsam::Pose3> odom_between_factor(
      X(key_ - 1), X(key_), pose_from.between(pose_to), between_noise);
    graph_.add(odom_between_factor);

    odom_diff = (pose_from.between(pose_to).matrix());
  }
  lidar_odom_buffer_.push_back(pose_to);

  return odom_diff;
}

void Optimization::add_velocity_factor(const gtsam::NavState & predict_state)
{
  gtsam::PriorFactor<gtsam::Vector3> predict_velocity(
    V(key_), predict_state.v(), gtsam::noiseModel::Isotropic::Precision(3, 1e3));
  graph_.add(predict_velocity);
}

Eigen::Matrix4d Optimization::update(const Eigen::Matrix4d & initial_pose, const double & score)
{
}

Eigen::Matrix4d Optimization::update(const Eigen::Matrix4d & initial_pose)
{
  gtsam::Pose3 initial_prior_pose(initial_pose);

  gtsam::Values initial_values;

  if (recovery_mode_) {
    // 回復モード中は、Odometryと目標姿勢の間を補間した位置を使用
    double progress = static_cast<double>(recovery_frames_ - 1) / max_recovery_frames_;

    // 現在のOdometryポーズ
    gtsam::Pose3 current_odom = gtsam::Pose3(initial_pose);

    // 差分の計算（Odometryの蓄積分を考慮）
    gtsam::Pose3 odom_delta = last_odom_pose_.between(current_odom);

    // 目標姿勢（Map Matching結果）に向かって徐々に補間
    gtsam::Pose3 interpolated_pose =
      last_odom_pose_.compose(odom_delta)
        .compose(gtsam::Pose3::Expmap(
          progress * gtsam::Pose3::Logmap(odom_delta.inverse().compose(
                       last_odom_pose_.inverse().compose(recovery_target_pose_)))));

    initial_values.insert(X(key_), interpolated_pose);
  } else {
    initial_values.insert(X(key_), initial_prior_pose);
  }

  smoother_ptr_->update(graph_, initial_values);
  smoother_ptr_->update();

  const auto result = smoother_ptr_->calculateEstimate();

  covariance_ = smoother_ptr_->marginalCovariance(gtsam::Symbol('x', key_));

  latest_state_ = gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), gtsam::Vector3::Zero());

  graph_.resize(0);
  key_++;

  return latest_state_.pose().matrix().cast<double>();
  // gtsam::Pose3 initial_prior_pose(initial_pose);

  // gtsam::Values initial_values;
  // initial_values.insert(X(key_), initial_prior_pose);
  // // initial_values.insert(V(key_), predict_state.v());

  // smoother_ptr_->update(graph_, initial_values);
  // smoother_ptr_->update();

  // const auto result = smoother_ptr_->calculateEstimate();

  // covariance_ = smoother_ptr_->marginalCovariance(gtsam::Symbol('x', key_));

  // latest_state_ = gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), gtsam::Vector3::Zero());

  // graph_.resize(0);
  // key_++;

  // return latest_state_.pose().matrix().cast<double>();
}
