#ifndef LIDAR_INERTIAL_ODOMETRY__MAP_MANAGER_HPP_
#define LIDAR_INERTIAL_ODOMETRY__MAP_MANAGER_HPP_

#include "lidar_inertial_odometry/submap.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <execution>

struct Voxel
{
  int x;
  int y;
  int z;

  bool operator==(const Voxel & voxel) const
  {
    return x == voxel.x && y == voxel.y && z == voxel.z;
  }
};

// Hash function
namespace std
{
template <>
struct hash<Voxel>
{
  size_t operator()(const Voxel & k) const
  {
    return ((size_t)k.x * 73856093) ^ ((size_t)k.y * 19349663) ^ ((size_t)k.z * 83492791);
  }
};
}  // namespace std

class MapManager
{
public:
  MapManager(const double resolution, const double distance_threshold)
  : resolution_(resolution), distance_threshold_(distance_threshold)
  {
    local_map_.reset(new PointCloud);
  }
  ~MapManager() = default;

  void add_points(const submap::Submap & submap)
  {
    const Eigen::Vector3d keyframe_position = submap.keyframe_pose.block<3, 1>(0, 3);

    for (auto & p : submap.map_points->points) {
      Voxel voxel = get_voxel_index(p.getArray3fMap());

      if (voxel_map_.find(voxel) == voxel_map_.end()) {
        voxel_map_[voxel] = p;
        local_map_->points.emplace_back(p);
      }
    }

    // remove voxel
    std::vector<Voxel> remove_voxel_lists;
    for (const auto & [voxel, point] : voxel_map_) {
      Eigen::Vector3d voxel_pos = point.getArray3fMap().cast<double>();
      if (distance_threshold_ < (voxel_pos - keyframe_position).norm()) {
        remove_voxel_lists.emplace_back(voxel);
      }
    }
    if (!remove_voxel_lists.empty()) {
      remove_map(remove_voxel_lists);
    }
  }

  void remove_map(const std::vector<Voxel> & keys)
  {
    local_map_->points.erase(
      std::remove_if(
        std::execution::par, local_map_->points.begin(), local_map_->points.end(),
        [&](const PointType & p) {
          Voxel key = get_voxel_index(p.getArray3fMap().cast<float>());
          return std::find(keys.begin(), keys.end(), key) != keys.end();
        }),
      local_map_->points.end());

    local_map_->width = local_map_->points.size();
    local_map_->height = 1;

    for (auto key : keys) {
      voxel_map_.erase(key);
    }
  }

  Voxel get_voxel_index(const Eigen::Vector3f & point)
  {
    Eigen::Vector3i voxel_index = (point / resolution_).array().floor().cast<int>();
    return {voxel_index.x(), voxel_index.y(), voxel_index.z()};
  }

  PointCloudPtr get_local_map() { return local_map_; }

private:
  double resolution_;
  double distance_threshold_;

  std::unordered_map<Voxel, PointType> voxel_map_;
  PointCloudPtr local_map_;
};

#endif
