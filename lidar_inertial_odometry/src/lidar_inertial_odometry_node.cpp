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
: Node("lidar_inertial_odometry", options), pose_buffer_(10)
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");

  // Smoother Type
  std::string smoother_type = this->declare_parameter<std::string>("smoother_type");
  if (smoother_type == "Kalman Filter") {
    smoother_type_ = SmootherType::KALMAN_FILTER;
  } else if (smoother_type == "Factor Graph") {
    smoother_type_ = SmootherType::FACTOR_GRAPH;
  } else {
    rclcpp::shutdown();
  }

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
  config.map_removal_distance = this->declare_parameter<double>("map_removal_distance");
  config.voxel_map_resolution = this->declare_parameter<double>("voxel_map_resolution");
  config.max_submap_size = this->declare_parameter<int>("max_submap_size");
  // IMU
  config.imu_calibration_time = this->declare_parameter<double>("imu_calibration_time");
  // ESKF Config
  config.acc_noise = this->declare_parameter<double>("acc_noise");
  config.gyro_noise = this->declare_parameter<double>("gyro_noise");
  config.acc_bias_noise = this->declare_parameter<double>("acc_bias_noise");
  config.gyro_bias_noise = this->declare_parameter<double>("gyro_bias_noise");
  config.translation_noise = this->declare_parameter<double>("translation_noise");
  config.rotation_noise = this->declare_parameter<double>("rotation_noise");
  config.gravity = this->declare_parameter<double>("gravity");
  lio_ = std::make_shared<LidarInertialOdometry>(config);

  // Point Cloud Pre-Processing
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
  pose_with_covariance_stamped_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose_with_covariance_stamped", 10);
  local_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "local_map", rclcpp::QoS{1}.transient_local());
  deskew_scan_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("deskew_scan", rclcpp::SensorDataQoS());
  odometry_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("odometry_path", 10);

  const double publish_frequency = this->declare_parameter<double>("publish_frequency");
  const double dt = 1.0 / std::max(publish_frequency, 0.1);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(dt),
    std::bind(&LidarInertialOdometryNode::process, this));
}

LidarInertialOdometryNode::~LidarInertialOdometryNode()
{
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

  // Scan Correction
  if (2 <= pose_buffer_.size()) {
    std::size_t size = pose_buffer_.size();
    Sophus::SE3d start_pose = pose_buffer_[size - 2];
    Sophus::SE3d end_pose = pose_buffer_[size - 1];
    auto delta_pose = (start_pose.inverse() * end_pose).log();

    tbb::parallel_for(
      tbb::blocked_range<std::size_t>(0, measurement.lidar_points.timestamp.size()),
      [&](const tbb::blocked_range<std::size_t> & range) {
        for (std::size_t i = range.begin(); i < range.end(); i++) {
          auto frame =
            measurement.lidar_points.raw_points->points[i].getVector3fMap().cast<double>();
          auto motion =
            Sophus::SE3d::exp((measurement.lidar_points.timestamp[i] - 0.5) * delta_pose);
          measurement.lidar_points.raw_points->points[i].getVector3fMap() =
            (motion * frame).cast<float>();
        }
      });
  }
  measurement.lidar_points.preprocessing_points =
    lio_->preprocessing(measurement.lidar_points.raw_points);

  // Predict & Update
  if (smoother_type_ == SmootherType::KALMAN_FILTER) {
    auto predict_states = lio_->predict(measurement);

    if (!lio_->update(measurement)) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to update.");
      return;
    }
  } else if (smoother_type_ == SmootherType::FACTOR_GRAPH) {
    auto predict_state = lio_->predict(measurement.lidar_points.stamp, measurement.imu_queue);

    if (!lio_->update(measurement, predict_state)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Measurement Update is Failed.");
      return;
    }
  }

  Eigen::Matrix4d estimated_pose = lio_->get_result();
  pose_buffer_.push_back(Sophus::SE3d(
    Eigen::Quaterniond(estimated_pose.block<3, 3>(0, 0)).normalized().toRotationMatrix(),
    estimated_pose.block<3, 1>(0, 3)));
  Eigen::Matrix<double, 6, 6> covariance = lio_->get_covariance();

  // Local map update
  if (lio_->update_local_map(estimated_pose, measurement.lidar_points)) {
    local_map_publisher_thread_ = std::thread(
      &LidarInertialOdometryNode::publish_local_map, this, measurement.lidar_points.stamp);
    local_map_publisher_thread_.detach();
  }

  // Publish ROS Message
  publisher_thread_ = std::thread(
    &LidarInertialOdometryNode::publish_message, this, measurement, estimated_pose, covariance);
  publisher_thread_.detach();
}

void LidarInertialOdometryNode::publish_local_map(const double stamp)
{
  PointCloudPtr transform_cloud(new PointCloud);
  transform_cloud = lio_->get_local_map();
  sensor_msgs::msg::PointCloud2 local_map_msg;
  pcl::toROSMsg(*transform_cloud, local_map_msg);
  local_map_msg.header.frame_id = map_frame_id_;
  local_map_msg.header.stamp = from_sec(stamp);
  local_map_publisher_->publish(local_map_msg);
}

void LidarInertialOdometryNode::publish_message(
  const sensor_type::Measurement & measurement, const Eigen::Matrix4d & pose,
  const Eigen::Matrix<double, 6, 6> & covariance)
{
  const auto current_time_stamp = from_sec(measurement.lidar_points.stamp);

  PointCloudPtr deskew_cloud(new PointCloud);
  pcl::transformPointCloud(*measurement.lidar_points.raw_points, *deskew_cloud, pose);
  sensor_msgs::msg::PointCloud2 deskew_cloud_msg;
  pcl::toROSMsg(*deskew_cloud, deskew_cloud_msg);
  deskew_cloud_msg.header.frame_id = map_frame_id_;
  deskew_cloud_msg.header.stamp = current_time_stamp;
  deskew_scan_publisher_->publish(deskew_cloud_msg);

  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.header.stamp = current_time_stamp;
  estimated_pose_msg.pose = lioamm_localizer_utils::convert_matrix_to_pose(pose.cast<float>());
  pose_stamped_publisher_->publish(estimated_pose_msg);

  geometry_msgs::msg::PoseWithCovarianceStamped estimated_pose_with_covariance;
  estimated_pose_with_covariance.header = estimated_pose_msg.header;
  estimated_pose_with_covariance.pose.pose = estimated_pose_msg.pose;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      estimated_pose_with_covariance.pose.covariance[i * 6 + j] = covariance(i, j);
    }
  }
  pose_with_covariance_stamped_publisher_->publish(estimated_pose_with_covariance);

  odometry_path_.poses.emplace_back(estimated_pose_msg);
  odometry_path_.header.frame_id = map_frame_id_;
  odometry_path_.header.stamp = current_time_stamp;
  odometry_path_publisher_->publish(odometry_path_);

  publish_tf(estimated_pose_msg.pose, current_time_stamp, map_frame_id_, base_frame_id_);
}

void LidarInertialOdometryNode::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloudPtr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);

  double scan_duration = 0.0;

  auto find_timestamp_field = [&msg]() -> std::string_view {
    constexpr std::array<std::string_view, 4> candidate_fields = {
      "t", "time", "timestamp", "time_stamp"};
    auto it = std::find_if(msg->fields.begin(), msg->fields.end(), [&](const auto & field) {
      return std::find(candidate_fields.begin(), candidate_fields.end(), field.name) !=
             candidate_fields.end();
    });
    return (it != msg->fields.end()) ? it->name : std::string_view{};
  };

  std::string_view timestamp_field = find_timestamp_field();
  std::vector<double> timestamps;
  if (!timestamp_field.empty()) {
    double coefficient = 1.0;

    auto process_timestamps = [&](auto t_iter) {
      for (; t_iter != t_iter.end(); ++t_iter) {
        double timestamp = static_cast<double>(*t_iter) * coefficient;
        timestamps.push_back(timestamp);
      }
    };

    if (timestamp_field == "t" || timestamp_field == "time_stamp") {
      coefficient = 1e-9f;
      process_timestamps(
        sensor_msgs::PointCloud2ConstIterator<std::uint32_t>(*msg, std::string(timestamp_field)));
    } else if (timestamp_field == "time") {
      process_timestamps(
        sensor_msgs::PointCloud2ConstIterator<float>(*msg, std::string(timestamp_field)));
    } else if (timestamp_field == "timestamp") {
      sensor_msgs::PointCloud2ConstIterator<double> t_iter(*msg, std::string(timestamp_field));
      if (t_iter != t_iter.end()) {
        double timestamp = *t_iter;
        coefficient = (timestamp > 1e14) ? 1e-9f : 1.0;
      }

      process_timestamps(
        sensor_msgs::PointCloud2ConstIterator<double>(*msg, std::string(timestamp_field)));
    }
    scan_duration = (timestamps.back() - timestamps.front());
  }

  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_sensor)) {
    return;
  }

  PointCloudPtr base_to_sensor_cloud(new PointCloud);
  pcl::transformPointCloud(
    *cloud, *base_to_sensor_cloud,
    lioamm_localizer_utils::convert_transform_to_matrix(base_to_sensor));

  sensor_type::Lidar new_points;
  new_points.stamp = rclcpp::Time(msg->header.stamp).seconds();
  new_points.timestamp = lioamm_localizer_utils::normalize(timestamps);
  new_points.lidar_start_time = new_points.stamp;
  new_points.lidar_end_time = new_points.lidar_start_time + scan_duration;
  new_points.raw_points = base_to_sensor_cloud;

  lio_->insert_points(new_points);
}

void LidarInertialOdometryNode::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped base_to_imu;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_imu)) {
    return;
  }

  Eigen::Transform<double, 3, Eigen::Affine> transform =
    lioamm_localizer_utils::get_eigen_transform(base_to_imu);

  sensor_type::Imu imu_data;
  imu_data.stamp = rclcpp::Time(msg->header.stamp).seconds();
  imu_data.linear_acceleration =
    transform *
    Eigen::Vector3d(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu_data.angular_velocity =
    transform *
    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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
