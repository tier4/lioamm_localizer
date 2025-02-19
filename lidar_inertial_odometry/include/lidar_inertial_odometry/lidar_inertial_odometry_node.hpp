#ifndef LIDAR_INERTIAL_ODOMETRY__LIDAR_INERTIAL_ODOMETRY_NODE_HPP_
#define LIDAR_INERTIAL_ODOMETRY__LIDAR_INERTIAL_ODOMETRY_NODE_HPP_

#include "lidar_inertial_odometry/imu_initializer.hpp"
#include "lidar_inertial_odometry/lidar_inertial_odometry.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class LidarInertialOdometryNode : public rclcpp::Node
{
public:
  explicit LidarInertialOdometryNode(const rclcpp::NodeOptions & options);
  ~LidarInertialOdometryNode();

  void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transformation);
  void publish_tf(
    const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);

  void main_thread();
  void process();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::shared_ptr<std::thread> thread_;
  std::shared_ptr<LidarInertialOdometry> lio_;

  std::string base_frame_id_;
  std::string map_frame_id_;
};

#endif
