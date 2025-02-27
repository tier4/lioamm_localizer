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

#include "lidar_inertial_odometry/lidar_inertial_odometry.hpp"

LidarInertialOdometry::LidarInertialOdometry(LidarInertialOdometry::LioConfig config)
: config_(config), transformation_(Eigen::Matrix4d::Identity())
{
  // Registration
  registration_ = std::make_shared<fast_gicp::FastGICP<PointType, PointType>>();
  // registration_->setResolution(config.resolution);
  registration_->setMaxCorrespondenceDistance(config.max_correspondence_distance);
  registration_->setCorrespondenceRandomness(config.correspondence_randomness);
  registration_->setTransformationEpsilon(config.transformation_epsilon);
  registration_->setMaximumIterations(config.max_iteration);
  // registration_->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
  registration_->setNumThreads(config.omp_num_thread);

  // ESKF
  eskf::ESKF::ESKFConfig eskf_config;
  eskf_config.acc_noise = config.acc_noise;
  eskf_config.gyro_noise = config.gyro_noise;
  eskf_config.acc_bias_noise = config.acc_bias_noise;
  eskf_config.gyro_bias_noise = config.gyro_bias_noise;
  eskf_config.translation_noise = config.translation_noise;
  eskf_config.rotation_noise = config.rotation_noise;
  eskf_ = std::make_shared<eskf::ESKF>(eskf_config);

  // IMU Initializer
  imu_ = std::make_shared<ImuInitializer>(config.imu_calibration_queue_size, config.gravity);

  // Map Manager
  map_manager_ =
    std::make_shared<MapManager>(config.voxel_map_resolution, config.map_removal_distance);

  // Optimization
  gtsam::ISAM2Params smoother_parameters;
  smoother_parameters.relinearizeThreshold = 0.1;
  smoother_parameters.relinearizeSkip = 1;
  // smoother_parameters.cacheLinearizedFactors = true;
  // smoother_parameters.findUnusedFactorSlots = true;
  // smoother_parameters.enableDetailedResults = false;
  smoother_parameters.factorization = gtsam::ISAM2Params::Factorization::QR;
  optimization_ = std::make_shared<Optimization>(smoother_parameters);
  optimization_->initialize();

  // IMU Integration
  gtsam::ISAM2Params imu_integ_parameters;
  imu_integ_parameters.relinearizeThreshold = 0.1;
  imu_integ_parameters.relinearizeSkip = 1;
  ImuIntegration::ImuConfig imu_config;
  imu_config.accel_noise_sigma = 0.05;  // 3.9939570888238808e-03;
  imu_config.gyro_noise_sigma = 0.02;   // 1.5636343949698187e-03;
  imu_config.pose_noise = 1e-2;
  imu_config.velocity_noise = 1e4;
  imu_config.bias_noise = 1e-5;
  imu_config.gravity = -9.80665;
  imu_config.reset_graph_key = 100;
  Eigen::VectorXd imu_bias(Eigen::VectorXd::Zero(6));
  imu_integration_ = std::make_shared<ImuIntegration>(imu_config, imu_bias, imu_integ_parameters);

  local_map_.reset(new PointCloud);
  keyframe_point_.reset(new PointCloud);
}

LidarInertialOdometry::~LidarInertialOdometry()
{
}

bool LidarInertialOdometry::sync_measurement(sensor_type::Measurement & measurement)
{
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  measurement.lidar_points = lidar_buffer_.front();
  lidar_buffer_.pop_front();

  while (!imu_buffer_.empty()) {
    auto imu = imu_buffer_.front();
    if (measurement.lidar_points.lidar_end_time < imu.stamp) {
      break;
    }

    measurement.imu_queue.push_back(imu);
    imu_buffer_.pop_front();
  }
  return true;
}

void LidarInertialOdometry::initialize(const sensor_type::Measurement & measurement)
{
  if (initialized_) {
    return;
  }

  for (auto imu : measurement.imu_queue) {
    imu_->add_imu(imu);
  }

  if (imu_->is_initialized()) {
    Eigen::Vector<double, 6> initial_imu_bias;
    initial_imu_bias.head<3>() = imu_->get_acc_mean();
    initial_imu_bias.tail<3>() = imu_->get_gyro_mean();

    Eigen::Vector3d gravity = imu_->get_gravity();

    Eigen::Matrix4d initial_pose(Eigen::Matrix4d::Identity());
    initial_pose.block<3, 3>(0, 0) = imu_->get_initial_orientation();

    set_timestamp(measurement.lidar_points.stamp, measurement.imu_queue.back().stamp);

    // Kalman Filter
    eskf_->initialize(initial_pose, initial_imu_bias, gravity, measurement.lidar_points.stamp);
    // eskf_->set_Q(imu_->get_acc_cov(), imu_->get_gyro_cov());

    // IMU Integration
    imu_integration_->initialize(
      initial_pose, initial_imu_bias, measurement.imu_queue.back().stamp);

    // Factor Graph Optimization
    optimization_->set_initial_value(
      initial_pose, initial_imu_bias, measurement.lidar_points.stamp);

    // Update Initial Local Map
    update_local_map(initial_pose, measurement.lidar_points);

    initialized_ = true;
  }
}

// IMU Integration
void LidarInertialOdometry::predict(std::deque<sensor_type::Imu> imu_queue)
{
  // Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();
  // Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();

  // for (auto imu : imu_queue) {
  //   const double dt = imu.stamp - last_imu_timestamp_;
  //   if (dt <= 0.0) {
  //     continue;
  //   }

  //   linear_acc = imu.linear_acceleration;
  //   angular_vel = imu.angular_velocity;
  //   imu_integration_->get_integrated_measurements().integrateMeasurement(
  //     linear_acc, angular_vel, dt);

  //   last_imu_timestamp_ = imu.stamp;
  // }

  // const auto [predict_state, predict_bias] = imu_integration_->predict(transformation_);

  // return std::make_tuple(predict_state, predict_bias);
  imu_integration_->integrate(imu_queue, optimization_->get_bias());
}

// ESKF
std::vector<Sophus::SE3d> LidarInertialOdometry::predict(sensor_type::Measurement & measurement)
{
  std::vector<Sophus::SE3d> imu_states;
  for (auto & imu : measurement.imu_queue) {
    imu.linear_acceleration *= imu_->get_imu_scale();
    eskf_->predict(imu);

    auto state = eskf_->get_state().get_x();
    imu_states.emplace_back(Sophus::SE3d(state.block<3, 3>(0, 0), state.block<3, 1>(0, 3)));
  }
  return imu_states;
}

// Factor Graph
bool LidarInertialOdometry::update(
  const sensor_type::Measurement & measurement, const gtsam::NavState & predict_state)
{
  auto lidar_points = measurement.lidar_points;

  Eigen::Matrix4d result_pose;
  if (!scan_matching(
        lidar_points.preprocessing_points, predict_state.pose().matrix().cast<double>(),
        result_pose)) {
    return false;
  }

  // factor graph optimization relative odometry pose
  // transformation_ = optimization_->update(lidar_points.stamp, result_pose,
  // measurement.map_pose_queue, predict_state);
  transformation_ = optimization_->update(
    lidar_points.stamp, result_pose, imu_integration_->get_integrated_measurements());

  return true;
}

// ESKF
bool LidarInertialOdometry::update(const sensor_type::Measurement & measurement)
{
  auto lidar_points = measurement.lidar_points;

  Eigen::Matrix4d initial_guess = eskf_->get_state().get_x();
  Eigen::Matrix4d result_pose;
  if (!scan_matching(lidar_points.preprocessing_points, initial_guess, result_pose)) {
    return false;
  }

  sensor_type::Pose pose_measurement;
  pose_measurement.stamp = lidar_points.stamp;
  pose_measurement.pose = result_pose;
  transformation_ = eskf_->update(pose_measurement);

  return true;
}

bool LidarInertialOdometry::scan_matching(
  const PointCloudPtr input_cloud_ptr, const Eigen::Matrix4d & initial_guess,
  Eigen::Matrix4d & result_pose)
{
  if (registration_->getInputTarget() == nullptr) {
    std::cerr << "not input target" << std::endl;
    return false;
  }

  registration_->setInputSource(input_cloud_ptr);

  PointCloudPtr align_cloud(new PointCloud);
  registration_->align(*align_cloud, initial_guess.cast<float>());

  if (!registration_->hasConverged()) {
    std::cerr << "not converged" << std::endl;
    return false;
  }

  result_pose = registration_->getFinalTransformation().cast<double>();

  return true;
}

bool LidarInertialOdometry::is_map_update_required(const Eigen::Matrix4d & pose)
{
  bool is_map_update = false;

  if (submaps_.empty()) {
    is_map_update = true;
  } else {
    PointType query_point;
    query_point.getVector3fMap() = pose.block<3, 1>(0, 3).cast<float>();

    const Eigen::Vector3d candidate_pos = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d candidate_rot = pose.block<3, 3>(0, 0);

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree_.nearestKSearch(query_point, 1, indices, distances)) {
      Eigen::Vector3d closest_pos = submaps_[indices[0]].keyframe_pose.block<3, 1>(0, 3);
      Eigen::Matrix3d closest_rot = submaps_[indices[0]].keyframe_pose.block<3, 3>(0, 0);

      const double delta_p = (candidate_pos - closest_pos).norm();
      const double delta_rot = Eigen::AngleAxisd(closest_rot.transpose() * candidate_rot).angle();

      if (config_.translation_threshold < delta_p || config_.rotation_threshold < delta_rot) {
        is_map_update = true;
      }
    }
  }

  return is_map_update;
}

bool LidarInertialOdometry::update_local_map(
  const Eigen::Matrix4d & pose, const sensor_type::Lidar & lidar_points)
{
  bool is_map_updated = false;

  if (is_map_update_required(pose)) {
    PointCloudPtr downsampling_submap_cloud(new PointCloud);
    map_voxel_grid_.setInputCloud(lidar_points.raw_points);
    map_voxel_grid_.filter(*downsampling_submap_cloud);

    submap::Submap submap(pose, downsampling_submap_cloud);
    submaps_.emplace_back(submap);

    // map_manager_->add_points(submap);
    // local_map_ = map_manager_->get_local_map();
    // registration_->setInputTarget(local_map_);
    // is_map_updated = true;

    PointType query_point;
    query_point.getVector3fMap() = pose.block<3, 1>(0, 3).cast<float>();
    keyframe_point_->points.emplace_back(query_point);
    kdtree_.setInputCloud(keyframe_point_);

    std::vector<int> indices;
    std::vector<float> distances;

    if (kdtree_.nearestKSearch(query_point, 20, indices, distances)) {
      PointCloudPtr new_map(new PointCloud);
      for (std::size_t idx = 0; idx < indices.size(); idx++) {
        *new_map += *submaps_[indices[idx]].map_points;
      }
      local_map_ = new_map;

      registration_->setInputTarget(new_map);

      is_map_updated = true;
    }
  }

  return is_map_updated;
}

PointCloudPtr LidarInertialOdometry::preprocessing(const PointCloudPtr & cloud_in)
{
  PointCloudPtr downsampling_cloud(new PointCloud);
  scan_voxel_grid_.setInputCloud(cloud_in);
  scan_voxel_grid_.filter(*downsampling_cloud);

  PointCloudPtr crop_cloud(new PointCloud);
  crop_.setInputCloud(downsampling_cloud);
  crop_.filter(*crop_cloud);

  return crop_cloud;
}
