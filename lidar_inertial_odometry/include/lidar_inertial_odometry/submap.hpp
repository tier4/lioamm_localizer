#ifndef LIDAR_INERTIAL_ODOMETRY__SUBMAP_HPP_
#define LIDAR_INERTIAL_ODOMETRY__SUBMAP_HPP_

#include "lioamm_localizer_common/point_type.hpp"

#include <pcl_ros/transforms.hpp>

namespace submap
{

struct Submap
{
  PointCloudPtr raw_points;
  PointCloudPtr map_points;
  Eigen::Matrix4d keyframe_pose;
  Submap()
  {
    keyframe_pose = Eigen::Matrix4d::Identity();
    raw_points.reset(new PointCloud);
    map_points.reset(new PointCloud);
  }
  Submap(const Eigen::Matrix4d & keyframe_pose, const PointCloudPtr & raw_points)
  {
    this->map_points.reset(new PointCloud);
    this->keyframe_pose = keyframe_pose;
    this->raw_points = raw_points;
    pcl::transformPointCloud(*this->raw_points, *this->map_points, this->keyframe_pose);
  }
};

}  // namespace submap

#endif
