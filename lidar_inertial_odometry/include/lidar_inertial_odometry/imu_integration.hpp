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
  ImuIntegration(const ImuConfig config, const Eigen::VectorXd & imu_bias) : config_(config)
  {
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params> params =
      gtsam::PreintegratedImuMeasurements::Params::MakeSharedD(config_.gravity);

    params->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * std::pow(config_.accel_noise_sigma, 2);
    params->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * std::pow(config_.gyro_noise_sigma, 2);
    params->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * std::pow(0.001, 2);
    // params->biasAccCovariance =
    //   gtsam::Matrix33::Identity(3, 3) * std::pow(imu_params.accel_bias_rw_sigma, 2);
    // params->biasOmegaCovariance =
    //   gtsam::Matrix33::Identity(3, 3) * std::pow(imu_params.gyro_bias_rw_sigma, 2);
    // params->biasAccOmegaInt = gtsam::Matrix::Identity(6, 6) * 1e-8;
    // params->use2ndOrderCoriolis = true;
    // params->omegaCoriolis = (gtsam::Vector(3) << -0.659 * -1 * imu_params.earth_rate, 0.0,
    //                         0.752 * -1 * imu_params.earth_rate)
    //                          .finished();

    const gtsam::imuBias::ConstantBias bias(imu_bias.head<3>(), imu_bias.tail<3>());
    imu_integrated_ptr_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(params, bias);
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

  void initialize(const double imu_time_stamp) { last_imu_time_stamp_ = imu_time_stamp; }

  gtsam::NavState predict(const gtsam::NavState & state, const gtsam::imuBias::ConstantBias & bias)
  {
    return imu_integrated_ptr_->predict(state, bias);
  }

  void insert_imu(const sensor_type::Imu & imu) { imu_queue_.push_back(imu); }

  gtsam::PreintegratedImuMeasurements & get_integrated_measurements() const
  {
    return *imu_integrated_ptr_;
  }

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_integrated_ptr_;

  ImuConfig config_;

  gtsam::NavState previous_state_;
  gtsam::imuBias::ConstantBias previous_bias_;

  ConcurrentQueue<sensor_type::Imu> imu_queue_;
  double last_imu_time_stamp_;
};

#endif
