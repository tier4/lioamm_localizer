#ifndef ESKF_LIO__ESKF_LIO_HPP_
#define ESKF_LIO__ESKF_LIO_HPP_

#include "eskf_lio/eskf.hpp"
#include "eskf_lio/sensor_type.hpp"
#include "eskf_lio/submap.hpp"

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/circular_buffer.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <deque>

namespace eskf_lio
{
class ESKFLio : public rclcpp::Node
{
public:
  ESKFLio(const rclcpp::NodeOptions & node_options);
  ~ESKFLio();

  void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  void initialized(
    const Eigen::Matrix4d & initial_pose, const Eigen::Vector<double, 6> & bias,
    const double & time_stamp);

  void run();
  bool sync(sensor_type::Measurement & measurement);

  bool scan_matching(const PointCloudPtr input_cloud_ptr, Eigen::Matrix4d & result);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transformation);
  void publish_tf(
    const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);

  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessing(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in);

  inline rclcpp::Time from_sec(const double timestamp)
  {
    const int seconds = static_cast<int>(timestamp);
    const int nanoseconds = static_cast<int>((timestamp - seconds * 1e9));

    return rclcpp::Time(seconds, nanoseconds);
  }

  inline double to_sec(const std_msgs::msg::Header & header)
  {
    return header.stamp.sec + header.stamp.nanosec / 1e9;
  }

  // points preprocessing
  inline void set_scan_voxel_size(const double voxel_size)
  {
    scan_voxel_grid_.setLeafSize(voxel_size, voxel_size, voxel_size);
  }
  inline void set_map_voxel_size(const double voxel_size)
  {
    map_voxel_grid_.setLeafSize(voxel_size, voxel_size, voxel_size);
  }
  inline void set_crop_area(const Eigen::Vector4f min, const Eigen::Vector4f max)
  {
    crop_.setMin(min);
    crop_.setMax(max);
  }

private:
  void update_keyframe(
    const Eigen::Matrix4d & keyframe_pose, const PointCloudPtr & keyframe_points);
  bool is_keyframe_update(const Eigen::Matrix4d & candidate_keyframe_pose);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::shared_ptr<eskf::ESKF> eskf_;

  std::shared_ptr<fast_gicp::FastVGICP<PointType, PointType>> registration_;

  pcl::KdTreeFLANN<PointType> kd_tree_;
  PointCloudPtr keyframes_;

  bool initialized_{false};

  std::vector<submap::Submap> submaps_;

  std::string base_frame_id_;
  std::string map_frame_id_;

  double keyframe_update_distance_threshold_;
  double keyframe_update_yaw_threshold_;

  pcl::VoxelGrid<pcl::PointXYZI> scan_voxel_grid_;
  pcl::VoxelGrid<pcl::PointXYZI> map_voxel_grid_;
  pcl::CropBox<pcl::PointXYZI> crop_;

  std::deque<sensor_type::PointCloud> points_queue_;
  std::deque<sensor_type::Imu> imu_queue_;

  double last_sensor_timestamp_;
};

}  // namespace eskf_lio

#endif
