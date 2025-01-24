#ifndef ESKF_LIO__SENSOR_TYPE_HPP_
#define ESKF_LIO__SENSOR_TYPE_HPP_

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>

namespace sensor_type
{

struct Imu
{
  double stamp;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

struct PointCloud
{
  double stamp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessing_points;
  PointCloud()
  {
    raw_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    preprocessing_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
};

struct Measurement
{
  sensor_type::PointCloud lidar_points;
  std::deque<sensor_type::Imu> imu_queue;
};

}  // namespace sensor_type

#endif
