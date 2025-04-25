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

#ifndef LIDAR_INERTIAL_ODOMETRY__OPTIMIZATION_HPP_
#define LIDAR_INERTIAL_ODOMETRY__OPTIMIZATION_HPP_

#include "lidar_inertial_odometry/imu_integration.hpp"

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

using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class Optimization
{
public:
  struct OptimizationParams
  {
    Eigen::VectorXd pose_noise;
    Eigen::Vector3d velocity_noise;
    Eigen::VectorXd bias_noise;
    double smoother_lag;
    OptimizationParams() {}
  };

  explicit Optimization(const gtsam::ISAM2Params parameter);
  ~Optimization() = default;

  [[nodiscard]] Eigen::Matrix4d update(
    const double & timestamp, const Eigen::Matrix4d & latest_frame,
    const std::deque<sensor_type::Pose> & map_pose_queue, const gtsam::NavState state);
  [[nodiscard]] Eigen::Matrix4d update(
    const double & timestamp, const Eigen::Matrix4d & lidar_pose_matrix,
    const gtsam::PreintegratedImuMeasurements & imu_integration_ptr);

  void set_initial_value(const Eigen::Matrix4d & initial_pose, const double & timestamp);
  void set_initial_value(
    const Eigen::Matrix4d & initial_pose, const Eigen::VectorXd & imu_bias,
    const double & timestamp);

  [[nodiscard]] gtsam::NavState get_state() { return latest_state_; }
  [[nodiscard]] gtsam::imuBias::ConstantBias get_bias() { return latest_imu_bias_; }

private:
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smoother_ptr_;
  std::shared_ptr<gtsam::ISAM2> optimizer_;

  gtsam::Vector imu_bias_noise_between_;

  gtsam::NavState latest_state_;
  gtsam::imuBias::ConstantBias latest_imu_bias_;

  boost::circular_buffer<gtsam::Pose3> odom_buffer_;

  std::size_t key_;
};

#endif
