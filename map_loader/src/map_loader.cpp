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

#include "lioamm_localizer_common/point_type.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace map_loader
{

class MapLoader : public rclcpp::Node
{
public:
  explicit MapLoader(const rclcpp::NodeOptions & options) : Node("map_loader", options)
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    this->declare_parameter<double>("leaf_size");

    const std::string map_path = this->declare_parameter<std::string>("map_path");
    const auto [map, centroid] = load_map(map_path);

    sensor_msgs::msg::PointCloud2 map_msgs;
    pcl::toROSMsg(*map, map_msgs);
    map_msgs.header.frame_id = "map";
    map_msgs.header.stamp = now();

    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS{1}.transient_local().keep_last(1));
    map_publisher_->publish(map_msgs);

    publish_tf(centroid);
  }
  ~MapLoader() {}

  void publish_tf(const Eigen::Vector4d & centroid)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "viewer";
    transform_stamped.transform.translation.x = centroid.x();
    transform_stamped.transform.translation.y = centroid.y();
    transform_stamped.transform.translation.z = centroid.z();
    transform_stamped.transform.rotation.w = 1.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;

    static_broadcaster_->sendTransform(transform_stamped);
  }

private:
  std::tuple<PointCloudPtr, Eigen::Vector4d> load_map(const std::string & map_path)
  {
    PointCloudPtr map_raw(new PointCloud);
    PointCloudPtr map_filtered(new PointCloud);

    if (pcl::io::loadPCDFile(map_path, *map_raw) == -1) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "map path is not exist. Can not load PCD map.");
      rclcpp::shutdown();
    }

    double leaf_size = 0.0;
    this->get_parameter("leaf_size", leaf_size);

    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(map_raw);
    voxel_grid.filter(*map_filtered);

    Eigen::Vector4d centroid(Eigen::Vector4d::Zero());
    pcl::compute3DCentroid(*map_filtered, centroid);

    return std::make_tuple(map_filtered, centroid);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

}  // namespace map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_loader::MapLoader)
