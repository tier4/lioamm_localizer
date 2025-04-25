#ifndef _RANGE_IMAGE_SCORE_HPP_
#define _RANGE_IMAGE_SCORE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZI;

struct RangeImageConfig
{
  double fov;
  double fov_up;
  double fov_down;
  int height;
  int width;
};

struct Pixel2D
{
  int x;
  int y;
};

class RangeImageScore : public rclcpp::Node
{
public:
  RangeImageScore();
  ~RangeImageScore() = default;

private:
  inline double radian(const double degree) { return degree / 180.0 / M_PI; }

  Pixel2D getPixelId(const PointType point, double & range);

  void downsample(
    const pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
    const double leaf_size);

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame);
  void transformPointCloud(
    pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
    const geometry_msgs::msg::TransformStamped frame_transform);

  geometry_msgs::msg::PoseStamped getLatestPose(const rclcpp::Time stamp);
  pcl::PointCloud<PointType>::Ptr getNearestMapCloud(const geometry_msgs::msg::Pose pose);

  void sensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  image_transport::Publisher debug_sensor_image_publisher_;
  image_transport::Publisher debug_map_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_map_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

  std::string base_frame_id_;
  double diff_threshold_;
  double distance_threshold_;
  double leaf_size_;
  double timeout_;

  RangeImageConfig config_;

  pcl::search::Search<PointType>::Ptr map_tree_;
  pcl::search::Search<PointType>::Ptr score_tree_;
  pcl::PointCloud<PointType>::Ptr map_;

  std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};
};

#endif
