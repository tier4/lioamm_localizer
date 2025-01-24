#ifndef ESKF_LIO__ESKF_HPP_
#define ESKF_LIO__ESKF_HPP_

#include "eskf_lio/sensor_type.hpp"
#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"
#include "lioamm_localizer_common/point_type.hpp"

#include <boost/circular_buffer.hpp>

namespace eskf
{

class ESKF
{
public:
  struct State
  {
    double stamp;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 18, 18> P = Eigen::Matrix<double, 18, 18>::Identity();

    State() {}
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

  ESKF(const ESKFConfig config, const int queue_size);
  ~ESKF();

  void initialize(
    const Eigen::Matrix4d initial_pose, const Eigen::Vector<double, 6> bias, const double stamp);

  void predict(sensor_type::Imu imu_measurement);
  bool update(const Eigen::Matrix4d & pose_measurement, const double stamp);

  inline Eigen::Matrix4d get_state()
  {
    State latest_state = state_buffer_.back();
    return lioamm_localizer_utils::get_matrix(latest_state.position, latest_state.quaternion);
  }

private:
  void reset(State & state, const Eigen::Vector<double, 18> & error);

  Eigen::Matrix<double, 18, 18> Fx_;
  Eigen::Matrix<double, 18, 12> Fi_;
  Eigen::Matrix<double, 12, 12> Qi_;
  Eigen::Matrix<double, 6, 6> V_;
  Eigen::Matrix<double, 18, 18> G_;

  ESKFConfig config_;

  boost::circular_buffer<State> state_buffer_;
};

}  // namespace eskf

#endif