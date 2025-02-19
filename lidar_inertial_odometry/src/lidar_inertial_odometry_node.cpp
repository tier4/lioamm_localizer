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

#include "lidar_inertial_odometry/lidar_inertial_odometry_node.hpp"

LidarInertialOdometryNode::LidarInertialOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_inertial_odometry", options)
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");

  LidarInertialOdometry::LioConfig config;
  // Registration Config
  config.max_iteration = this->declare_parameter<int>("max_iteration");
  config.omp_num_thread = this->declare_parameter<int>("omp_num_thread");
  config.correspondence_randomness = this->declare_parameter<int>("correspondence_randomness");
  config.transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
  config.max_correspondence_distance =
    this->declare_parameter<double>("max_correspondence_distance");
  config.resolution = this->declare_parameter<double>("resolution");
  // Local map config
  config.translation_threshold = this->declare_parameter<double>("translation_threshold");
  config.rotation_threshold = this->declare_parameter<double>("rotation_threshold") * M_PI / 180.0;
  // ESKF Config
  config.acc_noise = this->declare_parameter<double>("acc_noise");
  config.gyro_noise = this->declare_parameter<double>("gyro_noise");
  config.acc_bias_noise = this->declare_parameter<double>("acc_bias_noise");
  config.gyro_bias_noise = this->declare_parameter<double>("gyro_bias_noise");
  config.translation_noise = this->declare_parameter<double>("translation_noise");
  config.rotation_noise = this->declare_parameter<double>("rotation_noise");
  config.gravity = this->declare_parameter<double>("gravity");
  lio_ = std::make_shared<LidarInertialOdometry>(config);

  const double map_voxel_size = this->declare_parameter<double>("map_voxel_size");
  lio_->set_map_voxel_size(map_voxel_size);
  const double scan_voxel_size = this->declare_parameter<double>("scan_voxel_size");
  lio_->set_scan_voxel_size(scan_voxel_size);
  const double crop_min_x = this->declare_parameter<double>("crop_min_x");
  const double crop_min_y = this->declare_parameter<double>("crop_min_y");
  const double crop_min_z = this->declare_parameter<double>("crop_min_z");
  const double crop_max_x = this->declare_parameter<double>("crop_max_x");
  const double crop_max_y = this->declare_parameter<double>("crop_max_y");
  const double crop_max_z = this->declare_parameter<double>("crop_max_z");
  lio_->set_crop_area(
    Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 1.0),
    Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 1.0));

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  rclcpp::CallbackGroup::SharedPtr imu_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr points_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = imu_callback_group;
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;

  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS(),
    std::bind(&LidarInertialOdometryNode::callback_points, this, std::placeholders::_1),
    points_sub_opt);
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu_raw", rclcpp::SensorDataQoS(),
    std::bind(&LidarInertialOdometryNode::callback_imu, this, std::placeholders::_1), imu_sub_opt);

  pose_stamped_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 10);
  local_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "local_map", rclcpp::QoS{1}.transient_local());

  thread_ = std::make_shared<std::thread>(&LidarInertialOdometryNode::main_thread, this);
}

LidarInertialOdometryNode::~LidarInertialOdometryNode()
{
  if (thread_->joinable()) {
    thread_->join();
  }
}

void LidarInertialOdometryNode::main_thread()
{
  while (rclcpp::ok()) {
    process();
  }
  rclcpp::shutdown();
}

void LidarInertialOdometryNode::process()
{
  sensor_type::Measurement measurement;
  if (!lio_->sync_measurement(measurement)) {
    return;
  }

  if (!lio_->is_initialized()) {
    lio_->initialize(measurement);
    return;
  }

  // Predict
  lio_->predict(measurement);

  // Measurement Update
  if (!lio_->update(measurement)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to update.");
    return;
  }

  Eigen::Matrix4d estimated_pose = lio_->get_result();

  // Local map update
  if (lio_->update_local_map(estimated_pose, measurement.lidar_points)) {
    PointCloudPtr transform_cloud(new PointCloud);
    pcl::transformPointCloud(
      *measurement.lidar_points.raw_points, *transform_cloud, estimated_pose);
    sensor_msgs::msg::PointCloud2 local_map_msg;
    pcl::toROSMsg(*transform_cloud, local_map_msg);
    local_map_msg.header.frame_id = map_frame_id_;
    local_map_msg.header.stamp = now();
    local_map_publisher_->publish(local_map_msg);
  }

  const auto current_time_stamp = this->now();
  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.header.stamp = current_time_stamp;
  estimated_pose_msg.pose =
    lioamm_localizer_utils::convert_matrix_to_pose(estimated_pose.cast<float>());
  pose_stamped_publisher_->publish(estimated_pose_msg);

  publish_tf(estimated_pose_msg.pose, current_time_stamp, map_frame_id_, base_frame_id_);
}

void LidarInertialOdometryNode::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloudPtr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);

  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_sensor)) {
    return;
  }

  PointCloudPtr base_to_sensor_cloud(new PointCloud);
  pcl::transformPointCloud(
    *cloud, *base_to_sensor_cloud,
    lioamm_localizer_utils::convert_transform_to_matrix(base_to_sensor));

  PointCloudPtr preprocess_points(new PointCloud);
  preprocess_points = lio_->preprocessing(base_to_sensor_cloud);

  sensor_type::Lidar new_points;
  new_points.stamp = rclcpp::Time(msg->header.stamp).seconds();
  new_points.raw_points = base_to_sensor_cloud;
  new_points.preprocessing_points = preprocess_points;

  lio_->insert_points(new_points);
}

void LidarInertialOdometryNode::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // geometry_msgs::msg::TransformStamped base_to_imu;
  // if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_imu)) {
  //   return;
  // }

  // Eigen::Transform<double, 3, Eigen::Affine> transform =
  //   lioamm_localizer_utils::get_eigen_transform(base_to_imu);

  static sensor_msgs::msg::Imu last_imu = *msg;
  const double a = 0.8;
  const double b = 1.0 - a;

  sensor_msgs::msg::Imu imu_msg = *msg;
  // imu_msg.linear_acceleration.x =
  //   msg->linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
  // imu_msg.linear_acceleration.y =
  //   msg->linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
  // imu_msg.linear_acceleration.z =
  //   msg->linear_acceleration.z * a + last_imu.linear_acceleration.z * b;
  last_imu = *msg;

  sensor_type::Imu imu_data;
  imu_data.stamp = rclcpp::Time(imu_msg.header.stamp).seconds();
  imu_data.linear_acceleration = Eigen::Vector3d(
    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  imu_data.angular_velocity = Eigen::Vector3d(
    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

  lio_->insert_imu(imu_data);
}

bool LidarInertialOdometryNode::get_transform(
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

void LidarInertialOdometryNode::publish_tf(
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LidarInertialOdometryNode)
