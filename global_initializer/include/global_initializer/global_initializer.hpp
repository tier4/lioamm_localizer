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

#include "lioamm_localizer_common/point_type.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <teaser/fpfh.h>
#include <teaser/matcher.h>
#include <teaser/registration.h>

namespace global_initializer
{

using PointCloudFPFH = typename pcl::PointCloud<pcl::FPFHSignature33>;
using PointCloudFPFHPtr = typename PointCloudFPFH::Ptr;

class GlobalInitializer : public rclcpp::Node
{
public:
  explicit GlobalInitializer(const rclcpp::NodeOptions & node_options);
  ~GlobalInitializer() = default;

  PointCloudFPFHPtr extract_fpfh(const PointCloudPtr & cloud);

  void callback_global_map(const sensor_msgs::msg::PointCloud2 & msg);
  void callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg);
  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void process(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void downsample(
    const PointCloudPtr input_points_ptr, const PointCloudPtr & output_points_ptr,
    float downsample_leaf_size);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

  pcl::KdTreeFLANN<PointType> kd_tree_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pose_initialize_srv_;

  PointCloudPtr global_map_;
  sensor_msgs::msg::PointCloud2 sensor_points_msg;

  teaser::PointCloud target_cloud_;
  teaser::FPFHCloud::Ptr target_feature_;

  bool load_fpfh_feature_;
};

}  // namespace global_initializer

#endif
