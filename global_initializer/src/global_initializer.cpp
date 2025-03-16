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

#include "global_initializer/global_initializer.hpp"

namespace global_initializer
{

GlobalInitializer::GlobalInitializer(const rclcpp::NodeOptions & node_options)
: Node("global_pose_initializer", node_options)
{
  this->declare_parameter<double>("map_downsample_leaf_size");
  this->declare_parameter<double>("sensor_downsample_leaf_size");

  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");

  num_particles_ = this->declare_parameter<int>("num_particles");
  pos_noise_std_ = this->declare_parameter<double>("pos_noise_std");
  yaw_noise_std_ = this->declare_parameter<double>("yaw_noise_std");

  registration_ = std::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  const double transformation_epsilon = this->declare_parameter<double>("transformation_epsilon");
  const double step_size = this->declare_parameter<double>("step_size");
  const double resolution = this->declare_parameter<double>("resolution");
  const int max_iteration = this->declare_parameter<int>("max_iteration");
  const int omp_num_thread = this->declare_parameter<int>("omp_num_thread");
  registration_->setTransformationEpsilon(transformation_epsilon);
  registration_->setStepSize(step_size);
  registration_->setResolution(resolution);
  registration_->setMaximumIterations(max_iteration);
  registration_->setNeighborhoodSearchMethod(pclomp::KDTREE);
  if (0 < omp_num_thread) registration_->setNumThreads(omp_num_thread);

  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&GlobalInitializer::callback_sensor_points, this, std::placeholders::_1));
  global_map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&GlobalInitializer::callback_global_map, this, std::placeholders::_1));
  initial_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 5,
      std::bind(&GlobalInitializer::callback_initial_pose, this, std::placeholders::_1));

  pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose_3d", 5);
  particle_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_pose", 5);

  pose_initialize_srv_ = this->create_service<std_srvs::srv::Empty>(
    "global_initialize",
    std::bind(&GlobalInitializer::process, this, std::placeholders::_1, std::placeholders::_2));

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void GlobalInitializer::downsample(
  const PointCloudPtr input_points_ptr, const PointCloudPtr & output_points_ptr,
  float downsample_leaf_size)
{
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
  voxel_grid_filter.setInputCloud(input_points_ptr);
  voxel_grid_filter.filter(*output_points_ptr);
}

void GlobalInitializer::process(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  // TODO
  RCLCPP_INFO_STREAM(this->get_logger(), "global initialize...");
}

void GlobalInitializer::callback_global_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  global_map_.reset(new PointCloud);

  PointCloudPtr map(new PointCloud);
  pcl::fromROSMsg(msg, *map);

  double map_downsample_leaf_size;
  get_parameter<double>("map_downsample_leaf_size", map_downsample_leaf_size);
  downsample(map, global_map_, map_downsample_leaf_size);

  kd_tree_.setInputCloud(global_map_);
  registration_->setInputTarget(global_map_);

  RCLCPP_INFO_STREAM(this->get_logger(), "map loaded.");
}

void GlobalInitializer::callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(base_frame_id_, msg.header.frame_id, base_to_sensor)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get transform base_link to sensor_link.");
    return;
  }

  if (sensor_points_ptr_ == nullptr) {
    sensor_points_ptr_.reset(new PointCloud);
  }

  PointCloudPtr input_cloud_ptr(new PointCloud);
  pcl::fromROSMsg(msg, *input_cloud_ptr);

  PointCloudPtr base_to_sensor_points(new PointCloud);
  pcl::transformPointCloud(
    *input_cloud_ptr, *base_to_sensor_points,
    lioamm_localizer_utils::convert_transform_to_matrix(base_to_sensor));

  double sensor_downsample_leaf_size;
  get_parameter<double>("sensor_downsample_leaf_size", sensor_downsample_leaf_size);
  downsample(base_to_sensor_points, sensor_points_ptr_, sensor_downsample_leaf_size);
}

bool GlobalInitializer::estimator(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const PointCloudPtr & sensor_points_ptr,
  geometry_msgs::msg::PoseWithCovarianceStamped & estimated_pose)
{
  if (registration_->getInputTarget() == nullptr) {
    return false;
  }

  bool is_success = false;

  registration_->setInputSource(sensor_points_ptr);

  PointType query_point;
  query_point.x = pose.pose.pose.position.x;
  query_point.y = pose.pose.pose.position.y;
  query_point.z = pose.pose.pose.position.z;

  std::vector<int> idx(1);
  std::vector<float> dist(1);
  std::vector<RegistrationResult> results;
  std::vector<geometry_msgs::msg::Pose> initial_pose_lists;

  if (kd_tree_.nearestKSearch(query_point, 1, idx, dist)) {
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = query_point.x;
    initial_pose.position.y = query_point.y;
    initial_pose.position.z = global_map_->points[idx[0]].z;
    initial_pose.orientation = pose.pose.pose.orientation;

    Eigen::Vector3d euler =
      lioamm_localizer_utils::convert_quaternion_to_euler(initial_pose.orientation);

    std::random_device random;
    auto generator = std::mt19937(random());

    std::normal_distribution<double> position_dist(0.0, pos_noise_std_);
    std::normal_distribution<double> yaw_dist(0.0, yaw_noise_std_);
    for (int i = 0; i < num_particles_; i++) {
      geometry_msgs::msg::Pose particle_pose = initial_pose;
      particle_pose.position.x += position_dist(generator);
      particle_pose.position.y += position_dist(generator);

      Eigen::Vector3d initial_euler = euler;
      initial_euler.z() += yaw_dist(generator);

      auto initial_quaternion = lioamm_localizer_utils::convert_euler_to_quaternion(initial_euler);
      particle_pose.orientation.x = initial_quaternion.x();
      particle_pose.orientation.y = initial_quaternion.y();
      particle_pose.orientation.z = initial_quaternion.z();
      particle_pose.orientation.w = initial_quaternion.w();

      Eigen::Matrix4f initial_pose_matrix =
        lioamm_localizer_utils::convert_pose_to_matrix(particle_pose);

      PointCloudPtr output_cloud(new PointCloud);
      registration_->align(*output_cloud, initial_pose_matrix);
      if (!registration_->hasConverged()) {
        continue;
      }

      Eigen::Matrix4f transformation = registration_->getFinalTransformation();
      const double score = registration_->getNearestVoxelTransformationLikelihood();

      RegistrationResult result;
      result.score = score;
      result.pose = lioamm_localizer_utils::convert_matrix_to_pose(transformation);
      results.emplace_back(result);
      initial_pose_lists.emplace_back(particle_pose);
    }

    if (!results.empty()) {
      auto best_particle = std::max_element(
        results.begin(), results.end(),
        [](const RegistrationResult & a, const RegistrationResult & b) {
          return a.score < b.score;
        });

      geometry_msgs::msg::PoseArray particle_array;
      for (auto pose : initial_pose_lists) {
        particle_array.poses.emplace_back(pose);
      }
      particle_array.header = pose.header;
      particle_publisher_->publish(particle_array);

      estimated_pose = pose;
      estimated_pose.pose.pose = best_particle->pose;

      is_success = true;
    }
  }

  return is_success;
}

void GlobalInitializer::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "callback initial pose.");
  if (global_map_ == nullptr) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Global Map is not set.");
    return;
  }
  if (sensor_points_ptr_ == nullptr) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Sensor Points is not set.");
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "running initial pose search...");
  geometry_msgs::msg::PoseWithCovarianceStamped result_pose;
  if (estimator(*msg, sensor_points_ptr_, result_pose)) {
    pose_publisher_->publish(result_pose);
  }
}

bool GlobalInitializer::get_transform(
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

}  // namespace global_initializer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(global_initializer::GlobalInitializer)
