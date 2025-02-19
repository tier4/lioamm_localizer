#ifndef LIDAR_INERTIAL_ODOMETRY__LIDAR_INERTIAL_ODOMETRY_HPP_
#define LIDAR_INERTIAL_ODOMETRY__LIDAR_INERTIAL_ODOMETRY_HPP_

#include "lidar_inertial_odometry/eskf.hpp"
#include "lidar_inertial_odometry/imu_initializer.hpp"
#include "lidar_inertial_odometry/submap.hpp"
#include "lioamm_localizer_common/concurrent_queue.hpp"
#include "lioamm_localizer_common/sensor_type.hpp"

#include <fast_gicp/gicp/fast_vgicp.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pclomp/ndt_omp.h>

class LidarInertialOdometry
{
public:
  struct LioConfig
  {
    // FAST GICP
    double max_correspondence_distance;
    double transformation_epsilon;
    double correspondence_randomness;
    double resolution;
    int max_iteration;
    int omp_num_thread;

    // Local Map
    double translation_threshold;
    double rotation_threshold;

    // ESKF
    double acc_noise;
    double gyro_noise;
    double acc_bias_noise;
    double gyro_bias_noise;
    double translation_noise;
    double rotation_noise;
    double gravity;
  };

  LidarInertialOdometry(LioConfig config = LioConfig());
  ~LidarInertialOdometry();

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
    crop_.setNegative(true);
    crop_.setMin(min);
    crop_.setMax(max);
  }

  void initialize(const sensor_type::Measurement & measurement);

  void predict(sensor_type::Imu imu) { eskf_->predict(imu); }
  void predict(const sensor_type::Measurement & measurement);
  bool update(const sensor_type::Measurement & measurement);

  bool scan_matching(
    const PointCloudPtr input_cloud_ptr, const Eigen::Matrix4d & initial_guess,
    Eigen::Matrix4d & result_pose);

  bool is_map_update_required(const Eigen::Matrix4d & pose);
  bool update_local_map(const Eigen::Matrix4d & pose, const sensor_type::Lidar & lidar_points);

  inline void insert_points(const sensor_type::Lidar & points) { lidar_buffer_.push_back(points); }
  inline void insert_imu(const sensor_type::Imu & imu) { imu_buffer_.push_back(imu); }

  inline bool is_initialized() { return initialized_; }

  bool sync_measurement(sensor_type::Measurement & measurement);

  PointCloudPtr preprocessing(const PointCloudPtr & cloud_in);

  inline Eigen::Matrix4d get_result() { return transformation_; }

  inline PointCloudPtr get_local_map() { return local_map_; }

private:
  std::shared_ptr<ImuInitializer> imu_;
  std::shared_ptr<eskf::ESKF> eskf_;
  std::shared_ptr<fast_gicp::FastGICP<PointType, PointType>> registration_;

  ConcurrentQueue<sensor_type::Lidar> lidar_buffer_;
  ConcurrentQueue<sensor_type::Imu> imu_buffer_;

  pcl::VoxelGrid<PointType> scan_voxel_grid_;
  pcl::VoxelGrid<PointType> map_voxel_grid_;
  pcl::CropBox<PointType> crop_;

  PointCloudPtr local_map_;

  pcl::KdTreeFLANN<PointType> kdtree_;
  PointCloudPtr keyframe_point_;

  LioConfig config_;

  bool initialized_{false};

  Eigen::Matrix4d transformation_;

  std::vector<submap::Submap> submaps_;

  std::shared_ptr<sensor_type::Imu> last_imu_;
};

#endif
