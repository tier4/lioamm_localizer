#include "eskf_lio/eskf.hpp"

namespace eskf
{

ESKF::ESKF(ESKFConfig config, const int queue_size) : config_(config), state_buffer_(queue_size)
{
  Fx_ = Eigen::Matrix<double, 18, 18>::Identity();

  Fi_ = Eigen::Matrix<double, 18, 12>::Zero();
  Fi_.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Qi_ = Eigen::Matrix<double, 12, 12>::Identity();
  Qi_.block<3, 3>(0, 0) = std::pow(config_.acc_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(3, 3) = std::pow(config_.gyro_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(6, 6) = std::pow(config_.acc_bias_noise, 2) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(9, 9) = std::pow(config_.gyro_bias_noise, 2) * Eigen::Matrix3d::Identity();

  V_ = Eigen::Matrix<double, 6, 6>::Zero();
  V_.block<3, 3>(0, 0) = config_.translation_noise * Eigen::Matrix3d::Identity();
  V_.block<3, 3>(3, 3) = config_.rotation_noise * Eigen::Matrix3d::Identity();

  G_ = Eigen::Matrix<double, 18, 18>::Identity();
}

ESKF::~ESKF()
{
}

void ESKF::initialize(
  const Eigen::Matrix4d initial_pose, const Eigen::Vector<double, 6> bias, const double stamp)
{
  State initial_state;
  initial_state.stamp = stamp;
  initial_state.position = initial_pose.block<3, 1>(0, 3);
  initial_state.quaternion = Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0));
  initial_state.acc_bias = bias.head<3>();
  initial_state.gyro_bias = bias.tail<3>();
  initial_state.gravity << 0.0, 0.0, config_.gravity;
  state_buffer_.push_back(initial_state);
}

void ESKF::predict(sensor_type::Imu imu_measurement)
{
  State last_frame = state_buffer_.back();
  const double dt = imu_measurement.stamp - last_frame.stamp;
  if (dt < 0.0) return;

  State new_frame = last_frame;
  new_frame.stamp = imu_measurement.stamp;

  const Eigen::Matrix3d R = last_frame.quaternion.toRotationMatrix();
  const Eigen::Vector3d acc = imu_measurement.linear_acceleration - last_frame.acc_bias;
  const Eigen::Vector3d gyro = imu_measurement.angular_velocity - last_frame.gyro_bias;
  const Eigen::Vector3d angle_diff = gyro * dt;

  new_frame.position =
    last_frame.position + last_frame.velocity * dt + 0.5 * (R * acc + last_frame.gravity) * dt * dt;
  new_frame.velocity = last_frame.velocity + (R * acc + last_frame.gravity) * dt;
  new_frame.quaternion =
    last_frame.quaternion * lioamm_localizer_utils::convert_euler_to_quaternion(angle_diff);

  Fx_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(3, 6) = -R * lioamm_localizer_utils::skew_symmetric_matrix(acc) * dt;
  Fx_.block<3, 3>(3, 9) = -R * dt;
  Fx_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(6, 6) =
    lioamm_localizer_utils::convert_euler_to_quaternion(angle_diff).conjugate().toRotationMatrix();
  Fx_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  Qi_.block<3, 3>(0, 0) *= (dt * dt);
  Qi_.block<3, 3>(3, 3) *= (dt * dt);
  Qi_.block<3, 3>(6, 6) *= dt;
  Qi_.block<3, 3>(9, 9) *= dt;

  new_frame.P = Fx_ * last_frame.P * Fx_.transpose() + Fi_ * Qi_ * Fi_.transpose();

  state_buffer_.push_back(new_frame);
}

bool ESKF::update(const Eigen::Matrix4d & pose_measurement, const double stamp)
{
  State previous_state = state_buffer_.back();
  State latest_state = previous_state;
  latest_state.stamp = stamp;

  Eigen::Matrix4d previous_pose =
    lioamm_localizer_utils::get_matrix(previous_state.position, previous_state.quaternion);

  // calculate measurement error
  Eigen::Vector<double, 6> residual = Eigen::Vector<double, 6>::Zero(6);
  residual.head<3>() = pose_measurement.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3);
  residual.tail<3>() =
    (pose_measurement.block<3, 3>(0, 0).transpose() * previous_pose.block<3, 3>(0, 0))
      .eulerAngles(0, 1, 2);

  // calculation jacobian H
  // TODO: update hessian
  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 18, 6> K =
    previous_state.P * H.transpose() * (H * previous_state.P * H.transpose() + V_).inverse();
  Eigen::Vector<double, 18> error = K * residual;
  latest_state.P = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * previous_state.P;

  // reflect observation error
  latest_state.position += error.block<3, 1>(0, 0);
  latest_state.velocity += error.block<3, 1>(3, 0);
  latest_state.quaternion *=
    lioamm_localizer_utils::convert_euler_to_quaternion(error.block<3, 1>(6, 0));
  latest_state.acc_bias += error.block<3, 1>(9, 0);
  latest_state.acc_bias += error.block<3, 1>(12, 0);
  latest_state.gravity += error.block<3, 1>(15, 0);

  // reset ESKF
  reset(latest_state, error);

  state_buffer_.push_back(latest_state);

  return true;
}

void ESKF::reset(State & state, const Eigen::Vector<double, 18> & error)
{
  G_.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() -
    0.5 * lioamm_localizer_utils::skew_symmetric_matrix(error.block<3, 1>(6, 0));
  state.P = G_ * state.P * G_.transpose();
}

}  // namespace eskf
