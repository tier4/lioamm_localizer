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

#ifndef GLOBAL_OPTIMIZER__OPTIMIZATION_HPP_
#define GLOBAL_OPTIMIZER__OPTIMIZATION_HPP_

#include "lioamm_localizer_common/sensor_type.hpp"

#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <tuple>

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class Optimization
{
public:
  struct OptimizationParams
  {
    Eigen::VectorXd pose_prior_noise;
    Eigen::Vector3d velocity_prior_noise;
    Eigen::VectorXd bias_prior_noise;
    double smoother_lag;
    OptimizationParams() {}
  };

  explicit Optimization(const gtsam::ISAM2Params parameter);
  ~Optimization() = default;

  void add_map_matching_factor(const sensor_type::Pose & map_matching_pose, const double & score);
  void add_map_matching_factor(const sensor_type::Pose & map_matching_pose);
  void add_velocity_factor(const gtsam::NavState & predict_state);
  Eigen::Matrix4d add_odom_factor(
    const sensor_type::Pose & odom_pose, const bool & map_matching_is_fail);

  Eigen::Matrix4d update(const Eigen::Matrix4d & initial_pose);
  Eigen::Matrix4d update(const Eigen::Matrix4d & initial_pose, const double & score);

  Eigen::Matrix4d update(
    const double & timestamp, const Eigen::Matrix4d & lidar_pose_matrix,
    const gtsam::NavState & predict_state);

  void set_initial_value(const double & timestamp, const Eigen::Matrix4d & initial_pose);

  [[nodiscard]] gtsam::NavState get_state() { return latest_state_; }
  [[nodiscard]] gtsam::Matrix6 get_covariance() { return covariance_; }

  inline bool score_check_recovery(const double & score)
  {
    if (score_history_.size() < 3) return false;

    const double threshold = 1.5;  // NDTスコアの閾値

    // 現在のスコアが閾値以上
    bool current_score_is_good = (score >= threshold);

    // 過去数フレームがスコア不良だったかどうか
    bool had_score_failure = false;
    int consecutive_failures = 0;

    // 過去のスコアを検査し、連続して閾値以下だったフレーム数をカウント
    for (int i = score_history_.size() - 2; i >= 0; --i) {
      if (score_history_[i] < threshold) {
        consecutive_failures++;
      } else {
        break;  // 閾値以上のスコアがあった時点で終了
      }
    }

    // 連続5フレーム以上スコアが閾値以下だった後、回復した場合
    had_score_failure = (consecutive_failures >= 5);

    // 回復判定: 過去に十分なスコア不良期間があり、現在は良好
    return (had_score_failure && current_score_is_good);
  }

  inline void update_score_history(const double & score)
  {
    score_history_.push_back(score);
    if (score_history_.size() > 10) {
      score_history_.pop_front();
    }
  }

  inline gtsam::Pose3 blend_poses(
    const gtsam::Pose3 & pose1, const gtsam::Pose3 & pose2, double alpha)
  {
    // Alpha=0でpose1、Alpha=1でpose2
    gtsam::Pose3 delta = pose1.between(pose2);
    return pose1.compose(gtsam::Pose3::Expmap(alpha * gtsam::Pose3::Logmap(delta)));
  }

private:
  // Adaptive Map Matching Factor
  std::deque<double> score_history_;
  bool recovery_mode_{false};
  int recovery_frames_{0};
  int max_recovery_frames_{10};
  gtsam::Pose3 last_odom_pose_;
  gtsam::Pose3 recovery_target_pose_;
  bool recovery_initialized_ = false;  // 回復初期化フラグ
  gtsam::Pose3 odom_only_pose_;        // Odometryのみで推定した位置
  gtsam::Pose3 map_matching_pose_;     // Map Matchingで得られた位置
  double blend_progress_ = 0.0;        // ブレンド進行度（0.0～1.0）

  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smoother_ptr_;

  gtsam::NonlinearFactorGraph graph_;

  gtsam::NavState latest_state_;
  gtsam::Matrix6 covariance_;

  boost::circular_buffer<gtsam::Pose3> lidar_odom_buffer_;

  gtsam::ISAM2Params parameter_;

  std::size_t key_;

  // Adaptive Weight Manager
  bool map_matching_available_;
  double min_score_;
  double max_score_;
};

#endif
