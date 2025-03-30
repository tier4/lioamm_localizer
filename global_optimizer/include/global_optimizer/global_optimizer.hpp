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

#ifndef MAP_MATCHER__MAP_MATCHER_HPP_
#define MAP_MATCHER__MAP_MATCHER_HPP_

#include "global_optimizer/optimization.hpp"
#include "lioamm_localizer_common/concurrent_queue.hpp"
#include "lioamm_localizer_common/lioamm_localizer_utils.hpp"
#include "lioamm_localizer_common/point_type.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lioamm_localizer_msgs/msg/key_frame.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace global_optimizer
{

class GlobalOptimizer : public rclcpp::Node
{
public:
  explicit GlobalOptimizer(const rclcpp::NodeOptions & options);
  ~GlobalOptimizer();

private:
  void callback_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callback_keyframe(const lioamm_localizer_msgs::msg::KeyFrame::SharedPtr msg);

  void preprocessing(const PointCloudPtr & input_cloud_ptr, const PointCloudPtr & output_cloud_ptr);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transformation);
  void publish_tf(
    const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_;
  rclcpp::Subscription<lioamm_localizer_msgs::msg::KeyFrame>::SharedPtr keyframe_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matching_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_pose_path_publisher_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  pcl::VoxelGrid<PointType> voxel_grid_scan_;
  pcl::VoxelGrid<PointType> voxel_grid_map_;
  pcl::CropBox<PointType> crop_;

  std::string base_frame_id_;
  std::string map_frame_id_;

  PointCloudPtr map_;

  Eigen::Matrix4f transformation_;

  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> ndt_;

  std::shared_ptr<Optimization> optimizer_;

  nav_msgs::msg::Path global_pose_path_;

  bool is_initialized_{false};
  bool map_matching_fail_{false};

  int matching_fail_num_threshold_;
  double matching_score_threshold_;

  int fail_num_{0};
};

}  // namespace global_optimizer

#endif
