#ifndef ESKF_LIO__SENSOR_TYPE_HPP_
#define ESKF_LIO__SENSOR_TYPE_HPP_

#include "lioamm_localizer_common/point_type.hpp"

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

struct Lidar
{
  double stamp;
  PointCloudPtr raw_points;
  PointCloudPtr preprocessing_points;
  Lidar()
  {
    raw_points.reset(new PointCloud);
    preprocessing_points.reset(new PointCloud);
  }
};

struct Pose
{
  double stamp;
  Eigen::Matrix4d pose;
};

struct Measurement
{
  sensor_type::Lidar lidar_points;
  std::deque<sensor_type::Imu> imu_queue;
};

}  // namespace sensor_type

#endif
