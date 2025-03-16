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

#include "map_matcher/map_matcher.hpp"

namespace map_matcher
{

MapMatcher::MapMatcher(const rclcpp::NodeOptions & options)
: Node("map_matcher", options), transformation_(Eigen::Matrix4f::Identity())
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");

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

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&MapMatcher::callback_points, this, std::placeholders::_1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu_raw", 10, std::bind(&MapMatcher::callback_imu, this, std::placeholders::_1));
  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapMatcher::callback_map, this, std::placeholders::_1));
  odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "lidar_odometry", 10, std::bind(&MapMatcher::callback_odom, this, std::placeholders::_1));
  initial_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose_3d", 5,
      std::bind(&MapMatcher::callback_initial_pose, this, std::placeholders::_1));

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("map_pose", 10);
}

MapMatcher::~MapMatcher()
{
}

bool MapMatcher::integrate_imu(const std_msgs::msg::Header::_stamp_type & sensor_time_stamp)
{
  if (imu_queue_.empty()) return false;

  const double sensor_time_stamp_sec = sensor_time_stamp.sec + sensor_time_stamp.nanosec / 1e9;

  double imu_time_stamp = imu_queue_.front().stamp;
  while (!imu_queue_.empty() || imu_time_stamp < sensor_time_stamp_sec) {
    imu_time_stamp = imu_queue_.front().stamp;
    auto imu_data = imu_queue_.front();
    imu_queue_.pop_front();

    // integration
    const double dt = imu_data.stamp - last_imu_time_stamp_;
    if (dt <= 0.0) {
      continue;
    }

    imu_integration_->get_integrated_measurements().integrateMeasurement(
      imu_data.linear_acceleration, imu_data.angular_velocity, dt);

    last_imu_time_stamp_ = imu_data.stamp;
  }

  return true;
}

void MapMatcher::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (ndt_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Target map is not loaded.");
    return;
  }

  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_sensor)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get transform base_link to sensor_link.");
    return;
  }

  const auto lidar_time_stamp = msg->header.stamp;
  if (!integrate_imu(lidar_time_stamp)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to integrate imu measurement.");
    return;
  }

  const auto [predict_state, predict_bias] =
    imu_integration_->predict(transformation_.cast<double>());
  transformation_ = predict_state.pose().matrix().cast<float>();

  PointCloudPtr input_cloud(new PointCloud);
  PointCloudPtr base_to_sensor_points(new PointCloud);
  PointCloudPtr preprocessing_cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *input_cloud);

  pcl::transformPointCloud(
    *input_cloud, *base_to_sensor_points,
    lioamm_localizer_utils::convert_transform_to_matrix(base_to_sensor));
  preprocessing(base_to_sensor_points, preprocessing_cloud);

  ndt_->setInputSource(preprocessing_cloud);

  PointCloudPtr aligned_cloud(new PointCloud);
  ndt_->align(*aligned_cloud, transformation_);
  if (!ndt_->hasConverged()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Not Converged.");
    return;
  }

  transformation_ = ndt_->getFinalTransformation();
  const double score_nvtl = ndt_->getNearestVoxelTransformationLikelihood();
  const double score_tp = ndt_->getTransformationProbability();
  if (score_tp < 3.0) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Score is lower than threshold.");
    if (!map_matching_fail_) {
      map_matching_fail_ = true;
    } else {
      if (map_matching_fail_) {
        map_matching_fail_ = false;
      }
    }
  }
  const auto current_time_stamp = msg->header.stamp;
  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.stamp = lidar_time_stamp;
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.pose = lioamm_localizer_utils::convert_matrix_to_pose(transformation_);
  pose_publisher_->publish(estimated_pose_msg);

  publish_tf(estimated_pose_msg.pose, current_time_stamp, map_frame_id_, base_frame_id_);
}

void MapMatcher::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
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

  imu_queue_.push_back(imu_data);

  if (!is_initialized_) {
    last_imu_time_stamp_ = imu_data.stamp;
    is_initialized_ = true;
  }
}

void MapMatcher::callback_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

void MapMatcher::callback_odom(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (map_matching_fail_) {
    transformation_ = lioamm_localizer_utils::convert_pose_to_matrix(msg->pose);
  }
}

void MapMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initial pose callback.");

  transformation_ = lioamm_localizer_utils::convert_pose_to_matrix(msg->pose.pose);
  imu_integration_->initialize(transformation_.cast<double>(), Eigen::VectorXd::Zero(6));
}

void MapMatcher::preprocessing(
  const PointCloudPtr & input_cloud_ptr, const PointCloudPtr & output_cloud_ptr)
{
  PointCloudPtr downsampling_cloud(new PointCloud);
  voxel_grid_scan_.setInputCloud(input_cloud_ptr);
  voxel_grid_scan_.filter(*downsampling_cloud);

  crop_.setNegative(true);
  crop_.setInputCloud(downsampling_cloud);
  crop_.filter(*output_cloud_ptr);
}

void MapMatcher::publish_tf(
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

bool MapMatcher::get_transform(
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

}  // namespace map_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_matcher::MapMatcher)
