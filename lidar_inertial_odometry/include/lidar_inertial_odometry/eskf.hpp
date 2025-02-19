#ifndef LIDAR_INERTLA_ODOMETRY__ESKF_HPP_
#define LIDAR_INERTLA_ODOMETRY__ESKF_HPP_

#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"
#include "lioamm_localizer_common/point_type.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <sophus/so3.hpp>

#include <boost/circular_buffer.hpp>

namespace eskf
{

class ESKF
{
public:
  struct State
  {
    double stamp{0.0};
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 18, 18> P = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;

    State() {}

    Eigen::Matrix4d get_x() { return lioamm_localizer_utils::get_matrix(position, quaternion); }
  };
  struct ESKFConfig
  {
    // eskf
    double acc_noise;
    double gyro_noise;
    double acc_bias_noise;
    double gyro_bias_noise;

    double gravity;

    double translation_noise;
    double rotation_noise;
  };

  ESKF(const ESKFConfig config);
  ~ESKF();

  void initialize(
    const Eigen::Matrix4d initial_pose, const Eigen::Vector<double, 6> bias,
    const Eigen::Vector3d gravity, const double stamp);

  void predict(const sensor_type::Imu imu_measurement);
  Eigen::Matrix4d update(sensor_type::Pose & pose_measurement);

  void set_Q(const Eigen::Vector3d & acc_cov, const Eigen::Vector3d & gyro_cov);

  inline State get_state() { return state_; }
  inline void set_state(const State state) { state_ = state; }

private:
  Eigen::Matrix<double, 18, 18> Fx_;
  Eigen::Matrix<double, 18, 12> Fi_;
  Eigen::Matrix<double, 12, 12> Qi_;
  Eigen::Matrix<double, 6, 6> V_;
  Eigen::Matrix<double, 18, 18> G_;
  Eigen::Vector<double, 18> error_;

  ESKFConfig config_;

  State state_;
};

}  // namespace eskf

#endif
