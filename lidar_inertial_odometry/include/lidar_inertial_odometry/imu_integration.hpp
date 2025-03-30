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

#ifndef LIDAR_INERTIAL_ODOMETRY__IMU_INTEGRATION_HPP_
#define LIDAR_INERTIAL_ODOMETRY__IMU_INTEGRATION_HPP_

#include "lioamm_localizer_common/concurrent_queue.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <Eigen/Core>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <deque>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class ImuIntegration
{
public:
  struct ImuConfig
  {
    ImuConfig() {}

    // imu parameter
    double accel_noise_sigma;
    double gyro_noise_sigma;
    double accel_bias_rw_sigma;
    double gyro_bias_rw_sigma;
    double earth_rate;
    double gravity;

    // optimization parameter
    double pose_noise;
    double velocity_noise;
    double bias_noise;
    std::size_t reset_graph_key;
  };

  ImuIntegration() = default;
  ImuIntegration(
    const ImuConfig config, const Eigen::VectorXd & imu_bias, const gtsam::ISAM2Params parameter)
  : parameter_(parameter), config_(config), key_(0)
  {
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params> params =
      gtsam::PreintegratedImuMeasurements::Params::MakeSharedD(config_.gravity);

    params->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * std::pow(config_.accel_noise_sigma, 2);
    params->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * std::pow(config_.gyro_noise_sigma, 2);
    params->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * std::pow(0.001, 2);

    const gtsam::imuBias::ConstantBias bias(imu_bias.head<3>(), imu_bias.tail<3>());
    imu_integrated_ptr_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(params, bias);

    pose_noise_model_ =
      gtsam::noiseModel::Isotropic::Sigma(6, config_.pose_noise);  // rad,rad,rad,m, m, m
    velocity_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, config_.velocity_noise);  // m/s
    bias_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, config_.bias_noise);

    const double imu_acc_bias = 1.2123362494392119e-04;
    const double imu_gyro_bias = 8.6572985145653080e-05;
    imu_bias_noise_between_ = (gtsam::Vector(6) << imu_acc_bias, imu_acc_bias, imu_acc_bias,
                               imu_gyro_bias, imu_gyro_bias, imu_gyro_bias)
                                .finished();

    optimizer_ = std::make_shared<gtsam::ISAM2>(parameter_);
  }
  ~ImuIntegration() = default;

  void integrate(
    const double stamp, std::deque<sensor_type::Imu> imu_queue,
    const gtsam::imuBias::ConstantBias & bias)
  {
    imu_integrated_ptr_->resetIntegrationAndSetBias(bias);

    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

    for (const auto & imu : imu_queue) {
      const double dt = imu.stamp - last_imu_time_stamp_;
      if (dt <= 0.0) {
        continue;
      }
      linear_acceleration = imu.linear_acceleration;
      angular_velocity = imu.angular_velocity;
      imu_integrated_ptr_->integrateMeasurement(linear_acceleration, angular_velocity, dt);
      imu_queue.pop_front();

      last_imu_time_stamp_ = imu.stamp;
    }

    const double dt = stamp - last_imu_time_stamp_;
    if (0.0 < dt) {
      if (!imu_queue.empty()) {
        auto imu = imu_queue.front();
        linear_acceleration = imu.linear_acceleration;
        angular_velocity = imu.angular_velocity;
      }
      imu_integrated_ptr_->integrateMeasurement(linear_acceleration, angular_velocity, dt);
    }
  }

  void integrate(const double stamp, std::deque<sensor_type::Imu> imu_queue)
  {
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

    for (const auto & imu : imu_queue) {
      const double dt = imu.stamp - last_imu_time_stamp_;
      if (dt <= 0.0) {
        continue;
      }
      linear_acceleration = imu.linear_acceleration;
      angular_velocity = imu.angular_velocity;
      imu_integrated_ptr_->integrateMeasurement(linear_acceleration, angular_velocity, dt);
      imu_queue.pop_front();

      last_imu_time_stamp_ = imu.stamp;
    }

    const double dt = stamp - last_imu_time_stamp_;
    if (0.0 < dt) {
      if (!imu_queue.empty()) {
        auto imu = imu_queue.front();
        linear_acceleration = imu.linear_acceleration;
        angular_velocity = imu.angular_velocity;
      }
      imu_integrated_ptr_->integrateMeasurement(linear_acceleration, angular_velocity, dt);
    }
  }

  void initialize(
    const double stamp, const Eigen::Matrix4d & initial_pose, const Eigen::VectorXd & imu_bias)
  {
    key_ = 0;
    gtsam::imuBias::ConstantBias prior_imu_bias(imu_bias.head<3>(), imu_bias.tail<3>());
    gtsam::Pose3 prior_pose(initial_pose);
    gtsam::Vector3 prior_velocity = gtsam::Vector3::Zero();

    gtsam::Values initial_values;
    initial_values.insert(X(key_), prior_pose);
    initial_values.insert(V(key_), prior_velocity);
    initial_values.insert(B(key_), prior_imu_bias);

    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), prior_pose, pose_noise_model_));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(key_), prior_velocity, velocity_noise_model_));
    graph.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), prior_imu_bias, bias_noise_model_));

    optimizer_->update(graph, initial_values);

    last_imu_time_stamp_ = stamp;

    key_++;
  }

  void initialize(const double imu_time_stamp) { last_imu_time_stamp_ = imu_time_stamp; }

  gtsam::NavState predict(const gtsam::NavState & state, const gtsam::imuBias::ConstantBias & bias)
  {
    return imu_integrated_ptr_->predict(state, bias);
  }

  std::tuple<gtsam::NavState, gtsam::imuBias::ConstantBias> predict(
    const Eigen::Matrix4d & odometry)
  {
    if (key_ == config_.reset_graph_key) {
      gtsam::noiseModel::Gaussian::shared_ptr updated_pose_noise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer_->marginalCovariance(X(key_ - 1)));
      gtsam::noiseModel::Gaussian::shared_ptr updated_velocity_noise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer_->marginalCovariance(V(key_ - 1)));
      gtsam::noiseModel::Gaussian::shared_ptr updated_bias_noise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer_->marginalCovariance(B(key_ - 1)));
      optimizer_.reset(new gtsam::ISAM2(parameter_));

      gtsam::Values initial_values;
      initial_values.insert(X(0), previous_state_.pose());
      initial_values.insert(V(0), previous_state_.v());
      initial_values.insert(B(0), previous_bias_);

      gtsam::NonlinearFactorGraph graph;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), previous_state_.pose(), updated_pose_noise));
      graph.add(
        gtsam::PriorFactor<gtsam::Vector3>(V(0), previous_state_.v(), updated_velocity_noise));
      graph.add(
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), previous_bias_, updated_bias_noise));

      optimizer_->update(graph, initial_values);
      key_ = 1;
    }

    gtsam::NonlinearFactorGraph graph;

    auto & imu_integration =
      dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imu_integrated_ptr_);

    // IMU Factor
    gtsam::ImuFactor imu_factor(
      X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_), imu_integration);
    // IMU bias Factor
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> imu_bias_factor(
      B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(
        std::sqrt(imu_integrated_ptr_->deltaTij()) * imu_bias_noise_between_));
    // add IMU Factor
    graph.add(imu_factor);
    graph.add(imu_bias_factor);

    // Odometry Factor
    gtsam::Pose3 odometry_pose(odometry);
    gtsam::PriorFactor<gtsam::Pose3> odom_factor(
      X(key_), odometry_pose,
      gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()));
    // add Odometry Factor
    graph.add(odom_factor);

    auto prop_state = imu_integrated_ptr_->predict(previous_state_, previous_bias_);

    gtsam::Values initial_values;
    initial_values.insert(X(key_), prop_state.pose());
    initial_values.insert(V(key_), prop_state.v());
    initial_values.insert(B(key_), previous_bias_);

    optimizer_->update(graph, initial_values);
    optimizer_->update();

    const auto result = optimizer_->calculateEstimate();
    previous_state_ =
      gtsam::NavState(result.at<gtsam::Pose3>(X(key_)), result.at<gtsam::Vector3>(V(key_)));
    previous_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));

    imu_integrated_ptr_->resetIntegrationAndSetBias(previous_bias_);
    key_++;

    return std::make_tuple(previous_state_, previous_bias_);
  }

  void insert_imu(const sensor_type::Imu & imu) { imu_queue_.push_back(imu); }

  gtsam::PreintegratedImuMeasurements & get_integrated_measurements() const
  {
    return *imu_integrated_ptr_;
  }

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_integrated_ptr_;
  std::shared_ptr<gtsam::ISAM2> optimizer_;
  gtsam::ISAM2Params parameter_;

  ImuConfig config_;

  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_;
  gtsam::noiseModel::Isotropic::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Isotropic::shared_ptr bias_noise_model_;

  gtsam::Vector imu_bias_noise_between_;

  gtsam::NavState previous_state_;
  gtsam::imuBias::ConstantBias previous_bias_;

  ConcurrentQueue<sensor_type::Imu> imu_queue_;
  double last_imu_time_stamp_;

  std::size_t key_;
};

#endif
