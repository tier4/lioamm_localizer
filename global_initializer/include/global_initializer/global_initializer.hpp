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

#ifndef GLOBAL_INITIALIZER__GLOBAL_INITIALIZER_HPP_
#define GLOBAL_INITIALIZER__GLOBAL_INITIALIZER_HPP_

#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"
#include "lioamm_localizer_common/point_type.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <random>

struct RegistrationResult
{
  double score;
  geometry_msgs::msg::Pose pose;
};

namespace global_initializer
{

class GlobalInitializer : public rclcpp::Node
{
public:
  explicit GlobalInitializer(const rclcpp::NodeOptions & node_options);
  ~GlobalInitializer() = default;

  void callback_global_map(const sensor_msgs::msg::PointCloud2 & msg);
  void callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg);
  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void process(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  bool estimator(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const PointCloudPtr & sensor_points_ptr,
    geometry_msgs::msg::PoseWithCovarianceStamped & estimated_pose);

  void downsample(
    const PointCloudPtr input_points_ptr, const PointCloudPtr & output_points_ptr,
    float downsample_leaf_size);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transformation);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_publisher_;

  pcl::KdTreeFLANN<PointType> kd_tree_;
  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> registration_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pose_initialize_srv_;

  PointCloudPtr global_map_;
  PointCloudPtr sensor_points_ptr_;

  int num_particles_;
  double pos_noise_std_;
  double yaw_noise_std_;

  std::string base_frame_id_;
};

}  // namespace global_initializer

#endif
