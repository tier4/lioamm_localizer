#ifndef LIDAR_INERTIAL_ODOMETRY__OPTIMIZATION_HPP_
#define LIDAR_INERTIAL_ODOMETRY__OPTIMIZATION_HPP_

#include "lidar_inertial_odometry/imu_integration.hpp"

#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
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

  void initialize();

  [[nodiscard]] Eigen::Matrix4d update(
    const double & timestamp, const Eigen::Matrix4d & latest_frame,
    const std::deque<sensor_type::Pose> & map_pose_queue, const gtsam::NavState state,
    const gtsam::imuBias::ConstantBias imu_bias);

  void set_initial_value(const Eigen::Matrix4d & initial_pose, const double & timestamp);

private:
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smoother_ptr_;
  std::shared_ptr<gtsam::ISAM2> optimizer_;

  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_;
  gtsam::noiseModel::Isotropic::shared_ptr velocity_noise_model_;

  std::size_t key_;

  boost::circular_buffer<gtsam::Pose3> lidar_odom_buffer_;
};

#endif
