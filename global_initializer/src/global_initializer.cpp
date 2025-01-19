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
  this->declare_parameter<bool>("use_fpfh");
  this->declare_parameter<double>("normal_estimation.search_radius");
  this->declare_parameter<double>("fpfh.search_radius");
  this->declare_parameter<std::string>("fpfh_map_path");
  this->declare_parameter<bool>("load_fpfh_feature");
  this->get_parameter<bool>("load_fpfh_feature", load_fpfh_feature_);
  this->declare_parameter<double>("map_downsample_leaf_size");
  this->declare_parameter<double>("sensor_downsample_leaf_size");

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

  pose_initialize_srv_ = this->create_service<std_srvs::srv::Empty>(
    "global_initialize",
    std::bind(&GlobalInitializer::process, this, std::placeholders::_1, std::placeholders::_2));
}

PointCloudFPFHPtr GlobalInitializer::extract_fpfh(const PointCloudPtr & cloud)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "extract fpfh");
  double normal_estimation_radius;
  this->get_parameter<double>("normal_estimation.search_radius", normal_estimation_radius);
  double fpfh_search_radius;
  this->get_parameter<double>("fpfh.search_radius", fpfh_search_radius);

  RCLCPP_INFO_STREAM(this->get_logger(), "Normal Estimation");
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimation;
  normal_estimation.setRadiusSearch(normal_estimation_radius);
  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  normal_estimation.setSearchMethod(kdtree);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.compute(*normals);

  RCLCPP_INFO_STREAM(this->get_logger(), "FPFH Estimation");
  pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  PointCloudFPFHPtr features(new PointCloudFPFH);
  fpfh_estimation.setRadiusSearch(fpfh_search_radius);
  fpfh_estimation.setInputCloud(cloud);
  fpfh_estimation.setInputNormals(normals);
  fpfh_estimation.compute(*features);

  return features;
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
  RCLCPP_INFO_STREAM(this->get_logger(), "global initialize...");

  PointCloudPtr sensor_points(new PointCloud);
  PointCloudPtr filtered_points(new PointCloud);
  pcl::fromROSMsg(sensor_points_msg, *sensor_points);

  double sensor_downsample_leaf_size;
  get_parameter<double>("sensor_downsample_leaf_size", sensor_downsample_leaf_size);
  downsample(sensor_points, filtered_points, sensor_downsample_leaf_size);

  teaser::PointCloud source_cloud;
  for (std::size_t i = 0; i < filtered_points->size(); i++) {
    source_cloud.push_back(
      {filtered_points->at(i).x, filtered_points->at(i).y, filtered_points->at(i).z});
  }

  teaser::FPFHCloud::Ptr source_feature = extract_fpfh(filtered_points);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
    source_cloud, target_cloud_, *source_feature, *target_feature_, false, false, false, 0.95);

  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.5;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  RCLCPP_INFO_STREAM(this->get_logger(), "Solve Registration");
  teaser::RobustRegistrationSolver solver(params);
  solver.solve(source_cloud, target_cloud_, correspondences);
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish Solve");

  auto solution = solver.getSolution();
  auto translation = solution.translation;
  auto rotation = solution.rotation;
  Eigen::Quaterniond quaternion(rotation);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = now();
  pose_msg.pose.pose.position.x = translation.x();
  pose_msg.pose.pose.position.y = translation.y();
  pose_msg.pose.pose.position.z = translation.z();
  pose_msg.pose.pose.orientation.x = quaternion.x();
  pose_msg.pose.pose.orientation.y = quaternion.y();
  pose_msg.pose.pose.orientation.z = quaternion.z();
  pose_msg.pose.pose.orientation.w = quaternion.w();
  pose_publisher_->publish(pose_msg);
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

  bool use_fpfh;
  this->get_parameter<bool>("use_fpfh", use_fpfh);
  if (use_fpfh) {
    for (std::size_t i = 0; i < global_map_->size(); i++) {
      target_cloud_.push_back({global_map_->at(i).x, global_map_->at(i).y, global_map_->at(i).z});
    }

    std::string fpfh_map_path;
    get_parameter<std::string>("fpfh_map_path", fpfh_map_path);
    if (load_fpfh_feature_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Load FPFH Feature Map.");

      target_feature_.reset(new teaser::FPFHCloud);
      if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(fpfh_map_path, *target_feature_) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't read file: " << fpfh_map_path.c_str());
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Load Finish.");
    } else {
      target_feature_ = extract_fpfh(global_map_);
      RCLCPP_INFO_STREAM(this->get_logger(), "output fpfh feature");
      pcl::io::savePCDFile(fpfh_map_path, *target_feature_);
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "map loaded.");
}

void GlobalInitializer::callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_points_msg = msg;
}

void GlobalInitializer::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "callback initial pose.");
  if (global_map_ == nullptr) {
    return;
  }

  PointType query_point;
  query_point.x = msg->pose.pose.position.x;
  query_point.y = msg->pose.pose.position.y;
  query_point.z = msg->pose.pose.position.z;

  std::vector<int> idx(1);
  std::vector<float> dist(1);

  RCLCPP_INFO_STREAM(this->get_logger(), "running kdtree search...");
  if (kd_tree_.nearestKSearch(query_point, 1, idx, dist)) {
    geometry_msgs::msg::PoseWithCovarianceStamped result_pose;
    result_pose = *msg;
    result_pose.pose.pose.position.z = global_map_->points[idx[0]].z;
    pose_publisher_->publish(result_pose);
    RCLCPP_INFO_STREAM(this->get_logger(), "publish initial pose.");
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "can not found result of kdtree search.");
  }
}

}  // namespace global_initializer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(global_initializer::GlobalInitializer)
