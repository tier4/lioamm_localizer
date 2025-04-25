#ifndef GLOBAL_OPTIMIZER__OUTLIER_DETECTOR_HPP_
#define GLOBAL_OPTIMIZER__OUTLIER_DETECTOR_HPP_

#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"

#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

class OutlierDetector
{
public:
  OutlierDetector(const int max_history_size, const double mahalanobis_threshold)
  : pose_history_(max_history_size), mahalanobis_threshold_(mahalanobis_threshold)
  {
  }
  ~OutlierDetector() {}

  Eigen::VectorXd extract_diff_pose(
    const Eigen::Matrix4d & current_pose, const Eigen::Matrix4d & last_pose)
  {
    Eigen::Vector3d current_position = current_pose.block<3, 1>(0, 3);
    Eigen::Vector3d last_position = last_pose.block<3, 1>(0, 3);
    Eigen::Vector3d delta_position = current_position - last_position;

    Eigen::Matrix3d current_rotation = current_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d last_rotation = last_pose.block<3, 3>(0, 0);

    Eigen::Matrix3d delta_rotation = last_rotation.transpose() * current_rotation;
    Eigen::Quaterniond delta_quaternion(delta_rotation);
    delta_rotation.normalize();

    if (delta_quaternion.w() < 0) {
      delta_quaternion.coeffs() = -delta_quaternion.coeffs();
    }

    Eigen::VectorXd diff_pose(7);
    diff_pose << delta_position.x(), delta_position.y(), delta_position.z(), delta_quaternion.w(),
      delta_quaternion.x(), delta_quaternion.y(), delta_quaternion.z();

    return diff_pose;
  }

  void update()
  {
    if (pose_history_.size() < 2) {
      return;
    }

    // 特徴の次元
    int dim = pose_history_[0].size();

    // 重みを計算（新しいデータほど重みを大きくする）
    std::vector<double> weights(pose_history_.size());
    double sum_weights = 0.0;
    for (size_t i = 0; i < weights.size(); ++i) {
      // 最新のデータが最大の重みを持つ
      double age = static_cast<double>(weights.size() - 1 - i);
      weights[i] = std::exp(-time_decay_ * age);
      sum_weights += weights[i];
    }

    // 各重みを正規化
    for (auto & w : weights) {
      w /= sum_weights;
    }

    // 重み付き平均ベクトルを計算
    mean_ = Eigen::VectorXd::Zero(dim);
    for (size_t i = 0; i < pose_history_.size(); ++i) {
      mean_ += weights[i] * pose_history_[i];
    }

    // 重み付き共分散行列を計算
    covariance_ = Eigen::MatrixXd::Zero(dim, dim);
    for (size_t i = 0; i < pose_history_.size(); ++i) {
      Eigen::VectorXd centered = pose_history_[i] - mean_;
      covariance_ += weights[i] * (centered * centered.transpose());
    }

    // 数値安定性のために小さな値を追加
    double epsilon = 1e-6;
    covariance_ += Eigen::MatrixXd::Identity(dim, dim) * epsilon;
  }

  bool is_outlier(const Eigen::Matrix4d & pose_matrix)
  {
    if (!is_initialized_) {
      last_pose_ = pose_matrix;
      estimated_pose_ = pose_matrix;
      is_initialized_ = true;
      return false;
    }

    Eigen::VectorXd delta_pose = extract_diff_pose(pose_matrix, last_pose_);

    double distance = 0.0;
    if (2 <= pose_history_.size()) {
      Eigen::VectorXd centered = delta_pose - mean_;
      Eigen::MatrixXd cov_inv = covariance_.inverse();
      distance = std::sqrt((centered.transpose() * cov_inv * centered)(0, 0));
      std::cout << distance << std::endl;
    }

    bool is_outlier = (mahalanobis_threshold_ < distance);

    if (!is_outlier) {
      pose_history_.push_back(delta_pose);

      update();

      last_pose_ = pose_matrix;
      estimated_pose_ = pose_matrix;
    }

    return is_outlier;
  }

private:
  boost::circular_buffer<Eigen::VectorXd> pose_history_;

  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Vector<double, 6> mean_;

  Eigen::Matrix4d last_pose_;
  Eigen::Matrix4d estimated_pose_;

  double mahalanobis_threshold_;
  double time_decay_{0.1};
  bool is_initialized_;
};

#endif
