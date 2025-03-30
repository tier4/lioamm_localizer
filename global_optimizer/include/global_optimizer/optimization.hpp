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

  void add_map_matching_factor(const sensor_type::Pose & map_matching_pose);
  void add_odom_factor(
    const sensor_type::Pose & odom_pose, const bool & map_matching_is_fail = false);

  [[nodiscard]] Eigen::Matrix4d update(const Eigen::Matrix4d & initial_pose);

  [[nodiscard]] Eigen::Matrix4d update(
    const double & timestamp, const Eigen::Matrix4d & lidar_pose_matrix,
    const gtsam::NavState & predict_state);

  void set_initial_value(const double & timestamp, const Eigen::Matrix4d & initial_pose);

  [[nodiscard]] gtsam::NavState get_state() { return latest_state_; }
  [[nodiscard]] gtsam::Matrix6 get_covariance() { return covariance_; }

private:
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smoother_ptr_;

  gtsam::NonlinearFactorGraph graph_;

  gtsam::NavState latest_state_;
  gtsam::Matrix6 covariance_;

  boost::circular_buffer<gtsam::Pose3> lidar_odom_buffer_;

  gtsam::ISAM2Params parameter_;

  std::size_t key_;
};

#endif
