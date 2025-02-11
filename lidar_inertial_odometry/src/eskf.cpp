#include "lidar_inertial_odometry/eskf.hpp"

namespace eskf
{

ESKF::ESKF(ESKFConfig config, const int queue_size) : config_(config), state_buffer_(queue_size)
{
  Fx_ = Eigen::Matrix<double, 18, 18>::Identity();

  Fi_ = Eigen::Matrix<double, 18, 12>::Zero();
  Fi_.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Qi_ = Eigen::Matrix<double, 12, 12>::Identity();
  Qi_.block<3, 3>(0, 0) = std::pow(config_.acc_noise, 1) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(3, 3) = std::pow(config_.gyro_noise, 1) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(6, 6) = std::pow(config_.acc_bias_noise, 1) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(9, 9) = std::pow(config_.gyro_bias_noise, 1) * Eigen::Matrix3d::Identity();

  V_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> noise;
  noise << config_.translation_noise, config_.translation_noise, config_.translation_noise,
    config_.rotation_noise, config_.rotation_noise, config_.rotation_noise;
  V_ = noise.asDiagonal();

  G_ = Eigen::Matrix<double, 18, 18>::Identity();

  error_.setZero();
}

ESKF::~ESKF()
{
}

void ESKF::set_Q(const Eigen::Vector3d & acc_cov, const Eigen::Vector3d & gyro_cov)
{
  Qi_.block<3, 3>(0, 0) =
    Eigen::Matrix3d::Identity().array().colwise() * acc_cov.array().sqrt().array();
  Qi_.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity().array().colwise() * gyro_cov.array().sqrt().array();
  Qi_.block<3, 3>(6, 6) = std::pow(config_.acc_bias_noise, 1) * Eigen::Matrix3d::Identity();
  Qi_.block<3, 3>(9, 9) = std::pow(config_.gyro_bias_noise, 1) * Eigen::Matrix3d::Identity();
}

void ESKF::initialize(
  const Eigen::Matrix4d initial_pose, const Eigen::Vector<double, 6> bias,
  const Eigen::Vector3d gravity, const double stamp)
{
  State initial_state;
  initial_state.stamp = stamp;
  initial_state.position = initial_pose.block<3, 1>(0, 3);
  initial_state.velocity = Eigen::Vector3d::Zero();
  initial_state.quaternion = Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0));
  initial_state.acc_bias = bias.head<3>();
  initial_state.gyro_bias = bias.tail<3>();
  initial_state.gravity = gravity;
  initial_state.quaternion.normalize();
  state_ = initial_state;
  // state_buffer_.push_back(initial_state);
}

void ESKF::predict(sensor_type::Imu imu_measurement)
{
  const double dt = imu_measurement.stamp - state_.stamp;
  if (dt < 0.0) return;

  const Eigen::Matrix3d R = state_.quaternion.toRotationMatrix();
  const Eigen::Vector3d acc = imu_measurement.linear_acceleration - state_.acc_bias;
  const Eigen::Vector3d gyro = imu_measurement.angular_velocity - state_.gyro_bias;

  state_.position =
    state_.position + state_.velocity * dt + 0.5 * (R * acc + state_.gravity) * dt * dt;
  state_.velocity = state_.velocity + (R * acc + state_.gravity) * dt;
  state_.quaternion = R * (Sophus::SO3d::exp(gyro * dt).matrix());

  Fx_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(3, 6) = -R * lioamm_localizer_utils::skew_symmetric_matrix(acc) * dt;
  Fx_.block<3, 3>(3, 9) = -R * dt;
  Fx_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  Fx_.block<3, 3>(6, 6) = Sophus::SO3d::exp(gyro * dt).matrix().transpose();
  Fx_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  Qi_.block<3, 3>(0, 0) *= (dt * dt);
  Qi_.block<3, 3>(3, 3) *= (dt * dt);
  Qi_.block<3, 3>(6, 6) *= dt;
  Qi_.block<3, 3>(9, 9) *= dt;

  // error_ = Fx_ * error_;
  state_.P = Fx_ * state_.P * Fx_.transpose() + Fi_ * Qi_ * Fi_.transpose();
  state_.stamp = imu_measurement.stamp;

  // state_buffer_.push_back(new_frame);
}

Eigen::Matrix4d ESKF::update(sensor_type::Pose & pose_measurement)
{
  // calculate measurement error_
  Eigen::Vector<double, 6> residual = Eigen::Vector<double, 6>::Zero(6);
  residual.head<3>() = pose_measurement.pose.block<3, 1>(0, 3) - state_.position;
  residual.tail<3>() = lioamm_localizer_utils::convert_matrix_to_euler(
    state_.quaternion.toRotationMatrix().transpose() * pose_measurement.pose.block<3, 3>(0, 0));

  // calculation jacobian H
  // TODO: update hessian
  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
  /*
    Eigen::Matrix<double, 6, 19> Hx = Eigen::Matrix<double, 6, 19>::Zero();
    Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Hx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 19, 18> Xx = Eigen::Matrix<double, 19, 18>::Identity();
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << -previous_state.quaternion.x(), -previous_state.quaternion.y(),
      -previous_state.quaternion.z(), previous_state.quaternion.w(), -previous_state.quaternion.z(),
      previous_state.quaternion.y(), previous_state.quaternion.z(), previous_state.quaternion.w(),
      -previous_state.quaternion.x(), -previous_state.quaternion.y(), previous_state.quaternion.x(),
      previous_state.quaternion.w();
    Q_theta *= 0.5;
    Xx.block<4, 3>(6, 6) = Q_theta;

    H = Hx * Xx;
  */
  Eigen::Matrix<double, 18, 6> K =
    state_.P * H.transpose() * (H * state_.P * H.transpose() + V_).inverse();
  error_ = K * residual;
  state_.P = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * state_.P;
  // Eigen::Matrix<double, 18, 18> I = Eigen::Matrix<double, 18, 18>::Identity();
  // state_.P = (I - K * H) * state_.P * (I - K * H).transpose() + K * V_ * K.transpose();

  // reflect observation error_
  state_.position += error_.block<3, 1>(0, 0);
  state_.velocity += error_.block<3, 1>(3, 0);
  state_.quaternion *= Eigen::Quaterniond(Sophus::SO3d::exp(error_.block<3, 1>(6, 0)).matrix());
  // state_.quaternion.normalize();
  state_.acc_bias += error_.block<3, 1>(9, 0);
  state_.gyro_bias += error_.block<3, 1>(12, 0);
  state_.gravity += error_.block<3, 1>(15, 0);

  // reset ESKF
  G_.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() -
    0.5 * lioamm_localizer_utils::skew_symmetric_matrix(error_.block<3, 1>(6, 0));
  state_.P = G_ * state_.P * G_.transpose();
  error_.setZero();

  state_.stamp = pose_measurement.stamp;

  return lioamm_localizer_utils::get_matrix(state_.position, state_.quaternion);
}

}  // namespace eskf
