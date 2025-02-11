#ifndef LIDAR_INERTIAL_ODOMETRY__IMU_INTEGRATION_HPP_
#define LIDAR_INERTIAL_ODOMETRY__IMU_INTEGRATION_HPP_

#include "lioamm_localizer_common/sensor_type.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

class ImuIntegration
{
public:
  struct ImuConfig
  {
  };

  ImuIntegration() {}

  void integrate(const std::deque<sensor_type::Imu> & imu_queue) {}

  gtsam::PreintegratedImuMeasurements & get_integrated_measurements() const
  {
    return *imu_integrated_ptr_;
  }

private:
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_integrated_ptr_;
};

#endif
