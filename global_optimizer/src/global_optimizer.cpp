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

#include "global_optimizer/global_optimizer.hpp"

namespace global_optimizer
{

GlobalOptimizer::GlobalOptimizer(const rclcpp::NodeOptions & options)
: Node("global_optimizer", options), transformation_(Eigen::Matrix4f::Identity())
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");

  matching_score_threshold_ = this->declare_parameter<double>("matching_score_threshold");
  matching_fail_num_threshold_ = this->declare_parameter<int>("matching_fail_num_threshold");

  const double map_voxel_size = this->declare_parameter<double>("map_voxel_size");
  voxel_grid_map_.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
  const double scan_voxel_size = this->declare_parameter<double>("scan_voxel_size");
  voxel_grid_scan_.setLeafSize(scan_voxel_size, scan_voxel_size, scan_voxel_size);
  const double crop_min_x = this->declare_parameter<double>("crop_min_x");
  const double crop_min_y = this->declare_parameter<double>("crop_min_y");
  const double crop_min_z = this->declare_parameter<double>("crop_min_z");
  const double crop_max_x = this->declare_parameter<double>("crop_max_x");
  const double crop_max_y = this->declare_parameter<double>("crop_max_y");
  const double crop_max_z = this->declare_parameter<double>("crop_max_z");
  crop_.setMin(Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 1.0));
  crop_.setMax(Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 1.0));

  // Initialize Scan Registration
  ndt_ = std::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
  const double step_size = this->declare_parameter<double>("step_size");
  const double resolution = this->declare_parameter<double>("resolution");
  const int max_iteration = this->declare_parameter<int>("max_iteration");
  const int omp_num_thread = this->declare_parameter<int>("omp_num_thread");
  ndt_->setTransformationEpsilon(transformation_epsilon);
  ndt_->setStepSize(step_size);
  ndt_->setResolution(resolution);
  ndt_->setMaximumIterations(max_iteration);
  ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
  if (0 < omp_num_thread) ndt_->setNumThreads(omp_num_thread);

  // Factor Graph
  gtsam::ISAM2Params smoother_parameters;
  smoother_parameters.relinearizeThreshold = 0.01;
  smoother_parameters.relinearizeSkip = 1;
  // smoother_parameters.factorization = gtsam::ISAM2Params::Factorization::CHOLESKY;
  optimizer_ = std::make_shared<Optimization>(smoother_parameters);

  // Initialize IMU Pre-Integration
  gtsam::ISAM2Params imu_integ_parameters;
  imu_integ_parameters.relinearizeThreshold = 0.1;
  imu_integ_parameters.relinearizeSkip = 1;
  map_matcher::ImuIntegration::ImuConfig imu_config;
  imu_config.accel_noise_sigma = 0.05;
  imu_config.gyro_noise_sigma = 0.02;
  imu_config.pose_noise = 1e-2;
  imu_config.velocity_noise = 1e4;
  imu_config.bias_noise = 1e-5;
  imu_config.gravity = -9.80665;
  imu_config.reset_graph_key = 100;
  Eigen::VectorXd imu_bias(Eigen::VectorXd::Zero(6));
  imu_integration_ =
    std::make_shared<map_matcher::ImuIntegration>(imu_config, imu_bias, imu_integ_parameters);
  imu_integration_->initialize(transformation_.cast<double>(), Eigen::VectorXd::Zero(6));

  const int history_size = this->declare_parameter<int>("history_size");
  const double mahalanobis_threshold = this->declare_parameter<double>("mahalanobis_threshold");
  detector_ = std::make_shared<OutlierDetector>(history_size, mahalanobis_threshold);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&GlobalOptimizer::callback_map, this, std::placeholders::_1));
  initial_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose_3d", 5,
      std::bind(&GlobalOptimizer::callback_initial_pose, this, std::placeholders::_1));
  keyframe_subscriber_ = this->create_subscription<lioamm_localizer_msgs::msg::KeyFrame>(
    "keyframe", 5, std::bind(&GlobalOptimizer::callback_keyframe, this, std::placeholders::_1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu_raw", 10, std::bind(&GlobalOptimizer::callback_imu, this, std::placeholders::_1));

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("map_pose", 10);
  scan_matching_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("matching_pose", 10);
  scan_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_scan", rclcpp::SensorDataQoS());
  global_pose_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_pose_path", 10);
}

GlobalOptimizer::~GlobalOptimizer()
{
}

void GlobalOptimizer::callback_keyframe(const lioamm_localizer_msgs::msg::KeyFrame::SharedPtr msg)
{
  if (ndt_->getInputTarget() == nullptr) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Target map is not loaded.");
    return;
  }

  if (!is_initialized_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Initial Pose not received.");
    return;
  }

  const auto lidar_time_stamp = msg->header.stamp;

  // if (integrate_imu(lidar_time_stamp)) {
  //   RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to integrate imu measurement.");
  //   // return;
  //   const auto [predict_state, predict_bias] =
  //     imu_integration_->predict(transformation_.cast<double>());
  //   optimizer_->add_velocity_factor(predict_state);
  // }

  // TODO: check odom drift
  sensor_type::Pose odom_pose;
  odom_pose.stamp = rclcpp::Time(lidar_time_stamp).seconds();
  odom_pose.pose = lioamm_localizer_utils::convert_pose_to_matrix(msg->pose).cast<double>();

  // IMU
  // const auto [predict_state, predict_bias] =
  //   imu_integration_->predict(transformation_.cast<double>());
  // transformation_ = predict_state.pose().matrix().cast<float>();
  // optimizer_->add_velocity_factor(predict_state);

  PointCloudPtr input_cloud(new PointCloud);
  PointCloudPtr preprocessing_cloud(new PointCloud);
  pcl::fromROSMsg(msg->points, *input_cloud);

  preprocessing(input_cloud, preprocessing_cloud);

  ndt_->setInputSource(preprocessing_cloud);

  PointCloudPtr aligned_cloud(new PointCloud);
  ndt_->align(*aligned_cloud, transformation_.cast<float>());
  if (!ndt_->hasConverged()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Not Converged.");
    return;
  }

  auto scan_matching_pose = ndt_->getFinalTransformation();
  const double score_tp = ndt_->getTransformationProbability();
  // const double score_nvtl = ndt_->getNearestVoxelTransformationLikelihood();

  sensor_type::Pose scan_matching_result;
  scan_matching_result.pose = scan_matching_pose.cast<double>();
  scan_matching_result.stamp = rclcpp::Time(lidar_time_stamp).seconds();

  // TODO: check scan matching failure
  bool is_matching_score_low = (score_tp < matching_score_threshold_);
  if (is_matching_score_low) {
    fail_num_++;
  } else {
    fail_num_ = 0;
  }
  map_matching_fail_ = (matching_fail_num_threshold_ <= fail_num_);
  auto pose = scan_matching_result.pose.block<3, 1>(0, 3);
  std::cout << "map pose = (" << pose.x() << ", " << pose.y() << ")" << std::endl;
  if (!is_matching_score_low) {
    // if (!detector_->is_outlier(scan_matching_result.pose)) {
    optimizer_->add_map_matching_factor(scan_matching_result);
    geometry_msgs::msg::PoseStamped scan_matching_pose_msgs;
    scan_matching_pose_msgs.header.frame_id = map_frame_id_;
    scan_matching_pose_msgs.header.stamp = lidar_time_stamp;
    scan_matching_pose_msgs.pose =
      lioamm_localizer_utils::convert_matrix_to_pose(scan_matching_pose);
    scan_matching_pose_publisher_->publish(scan_matching_pose_msgs);
    // }
  } else {
    // optimizer_->update_score_history(score_tp);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Matching failed.");
  }
  auto odom_diff = optimizer_->add_odom_factor(odom_pose, map_matching_fail_);

  auto result = optimizer_->update(transformation_.cast<double>());
  transformation_ = result.cast<float>();
  auto opt_pose = transformation_.block<3, 1>(0, 3);
  std::cout << "optimization pose = (" << opt_pose.x() << ", " << opt_pose.y() << ")" << std::endl;

  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.stamp = this->now();
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.pose = lioamm_localizer_utils::convert_matrix_to_pose(result.cast<float>());
  pose_publisher_->publish(estimated_pose_msg);

  sensor_msgs::msg::PointCloud2 filtered_scan_msg;
  PointCloudPtr transformed_cloud(new PointCloud);
  pcl::transformPointCloud(*input_cloud, *transformed_cloud, transformation_);
  pcl::toROSMsg(*transformed_cloud, filtered_scan_msg);
  filtered_scan_msg.header.stamp = this->now();
  filtered_scan_msg.header.frame_id = map_frame_id_;
  scan_publisher_->publish(filtered_scan_msg);

  global_pose_path_.header.frame_id = map_frame_id_;
  global_pose_path_.header.stamp = this->now();
  global_pose_path_.poses.emplace_back(estimated_pose_msg);
  global_pose_path_publisher_->publish(global_pose_path_);

  publish_tf(estimated_pose_msg.pose, this->now(), map_frame_id_, base_frame_id_);
}

void GlobalOptimizer::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped base_to_imu;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_imu)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get transform base_link to imu_link.");
    return;
  }

  Eigen::Transform<double, 3, Eigen::Affine> transform_matrix =
    lioamm_localizer_utils::get_eigen_transform(base_to_imu);

  sensor_type::Imu imu_data;
  imu_data.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  imu_data.linear_acceleration =
    transform_matrix *
    Eigen::Vector3d(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu_data.angular_velocity =
    transform_matrix *
    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  // imu_queue_.push_back(imu_data);

  if (!is_imu_initialized_) {
    last_imu_time_stamp_ = imu_data.stamp;
    is_imu_initialized_ = true;
  }
}

void GlobalOptimizer::callback_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (map_ == nullptr) {
    map_.reset(new PointCloud);
  }

  PointCloudPtr map_raw(new PointCloud);
  pcl::fromROSMsg(*msg, *map_raw);

  voxel_grid_map_.setInputCloud(map_raw);
  voxel_grid_map_.filter(*map_);

  ndt_->setInputTarget(map_raw);
}

void GlobalOptimizer::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initial pose callback.");

  transformation_ = lioamm_localizer_utils::convert_pose_to_matrix(msg->pose.pose);
  is_initialized_ = true;

  optimizer_->set_initial_value(
    rclcpp::Time(msg->header.stamp).seconds(), transformation_.cast<double>());
  imu_integration_->initialize(transformation_.cast<double>(), Eigen::VectorXd::Zero(6));
}

void GlobalOptimizer::preprocessing(
  const PointCloudPtr & input_cloud_ptr, const PointCloudPtr & output_cloud_ptr)
{
  PointCloudPtr downsampling_cloud(new PointCloud);
  voxel_grid_scan_.setInputCloud(input_cloud_ptr);
  voxel_grid_scan_.filter(*downsampling_cloud);

  crop_.setNegative(true);
  crop_.setInputCloud(downsampling_cloud);
  crop_.filter(*output_cloud_ptr);
}

bool GlobalOptimizer::integrate_imu(const std_msgs::msg::Header::_stamp_type & sensor_time_stamp)
{
  if (imu_queue_.empty()) return false;

  const double sensor_time_stamp_sec = sensor_time_stamp.sec + sensor_time_stamp.nanosec / 1e9;

  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  while (!imu_queue_.empty()) {
    auto imu = imu_queue_.front();
    if (sensor_time_stamp_sec < imu.stamp) {
      break;
    }
    imu_queue_.pop_front();

    // integration
    const double dt = imu.stamp - last_imu_time_stamp_;
    if (dt <= 0.0) {
      continue;
    }

    linear_acceleration = imu.linear_acceleration;
    angular_velocity = imu.angular_velocity;

    imu_integration_->get_integrated_measurements().integrateMeasurement(
      linear_acceleration, angular_velocity, dt);

    last_imu_time_stamp_ = imu.stamp;
  }

  const double dt = sensor_time_stamp_sec - last_imu_time_stamp_;
  if (0.0 < dt) {
    if (!imu_queue_.empty()) {
      auto imu = imu_queue_.front();
      linear_acceleration = imu.linear_acceleration;
      angular_velocity = imu.angular_velocity;
    }
    imu_integration_->get_integrated_measurements().integrateMeasurement(
      linear_acceleration, angular_velocity, dt);
  }

  return true;
}

void GlobalOptimizer::publish_tf(
  const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
  const std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;

  broadcaster_->sendTransform(transform_stamped);
}

bool GlobalOptimizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transformation)
{
  try {
    transformation = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

}  // namespace global_optimizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(global_optimizer::GlobalOptimizer)
