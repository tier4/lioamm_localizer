#include "eskf_lio/eskf_lio.hpp"

namespace eskf_lio
{

ESKFLio::ESKFLio(const rclcpp::NodeOptions & node_options) : Node("eskf_lio", node_options)
{
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");

  keyframe_update_distance_threshold_ =
    this->declare_parameter<double>("keyframe_update_distance_threshold");
  keyframe_update_yaw_threshold_ = this->declare_parameter<double>("keyframe_update_yaw_threshold");

  // Scan Registration Parameter
  registration_ = std::make_shared<fast_gicp::FastVGICP<PointType, PointType>>();
  registration_->setMaxCorrespondenceDistance(
    this->declare_parameter<double>("max_correspondence_distance"));
  registration_->setTransformationEpsilon(
    this->declare_parameter<double>("transformation_epsilon"));
  registration_->setCorrespondenceRandomness(
    this->declare_parameter<int>("correspondence_randomness"));
  registration_->setMaximumIterations(this->declare_parameter<int>("max_iteration"));
  registration_->setNumThreads(this->declare_parameter<int>("omp_num_thread"));

  // Pre Processing Parameter
  const double map_voxel_size = this->declare_parameter<double>("map_voxel_size");
  set_map_voxel_size(map_voxel_size);
  const double scan_voxel_size = this->declare_parameter<double>("scan_voxel_size");
  set_scan_voxel_size(scan_voxel_size);
  const double crop_min_x = this->declare_parameter<double>("crop_min_x");
  const double crop_min_y = this->declare_parameter<double>("crop_min_y");
  const double crop_min_z = this->declare_parameter<double>("crop_min_z");
  const double crop_max_x = this->declare_parameter<double>("crop_max_x");
  const double crop_max_y = this->declare_parameter<double>("crop_max_y");
  const double crop_max_z = this->declare_parameter<double>("crop_max_z");
  set_crop_area(
    Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 1.0),
    Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 1.0));

  // ESKF Parameter
  eskf::ESKF::ESKFConfig eskf_config;
  eskf_config.acc_noise = this->declare_parameter<double>("acc_noise");
  eskf_config.gyro_noise = this->declare_parameter<double>("gyro_noise");
  eskf_config.acc_bias_noise = this->declare_parameter<double>("acc_bias_noise");
  eskf_config.gyro_bias_noise = this->declare_parameter<double>("gyro_bias_noise");
  eskf_config.translation_noise = this->declare_parameter<double>("translation_noise");
  eskf_config.rotation_noise = this->declare_parameter<double>("rotation_noise");
  eskf_config.gravity = this->declare_parameter<double>("gravity");
  eskf_ = std::make_shared<eskf::ESKF>(eskf_config, 10);

  this->declare_parameter<std::vector<double>>("acc_bias");
  this->declare_parameter<std::vector<double>>("gyro_bias");

  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS(),
    std::bind(&ESKFLio::callback_points, this, std::placeholders::_1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu_raw", 5, std::bind(&ESKFLio::callback_imu, this, std::placeholders::_1));
  pose_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 5);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ESKFLio::run, this));
}

ESKFLio::~ESKFLio()
{
}

void ESKFLio::run()
{
  sensor_type::Measurement measurement;
  if (!sync(measurement)) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Failed to sync sensor measurement.");
    return;
  }

  if (!initialized_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized ESKF Lio.");

    auto acc_bias = this->get_parameter("acc_bias").as_double_array();
    auto gyro_bias = this->get_parameter("gyro_bias").as_double_array();
    eskf_->initialize(
      Eigen::Matrix4d::Identity(),
      Eigen::Vector<double, 6>(
        acc_bias[0], acc_bias[1], acc_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]),
      measurement.lidar_points.stamp);
    initialized_ = true;

    return;
  }

  for (auto & imu : measurement.imu_queue) {
    eskf_->predict(imu);
  }

  Eigen::Matrix4d scan_matching_pose;
  if (!scan_matching(measurement.lidar_points.preprocessing_points, scan_matching_pose)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to scan matching.");
    return;
  }

  if (!eskf_->update(scan_matching_pose, measurement.lidar_points.stamp)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to update.");
    return;
  }

  const Eigen::Matrix4d estimated_pose = eskf_->get_state();

  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.header.stamp = from_sec(measurement.lidar_points.stamp);
  estimated_pose_msg.pose =
    lioamm_localizer_utils::convert_matrix_to_pose(estimated_pose.cast<float>());
  pose_stamped_publisher_->publish(estimated_pose_msg);

  publish_tf(
    estimated_pose_msg.pose, estimated_pose_msg.header.stamp, map_frame_id_, base_frame_id_);
}

bool ESKFLio::scan_matching(const PointCloudPtr input_cloud_ptr, Eigen::Matrix4d & result)
{
  if (!registration_->getInputTarget()) {
    return false;
  }

  registration_->setInputSource(input_cloud_ptr);

  PointCloudPtr align_cloud(new PointCloud);
  registration_->align(*align_cloud);

  if (!registration_->hasConverged()) {
    return false;
  }

  result = registration_->getFinalTransformation().cast<double>();

  return true;
}

bool ESKFLio::sync(sensor_type::Measurement & measurement)
{
  if (points_queue_.empty() || imu_queue_.empty()) {
    return false;
  }

  auto sensor_points = points_queue_.front();
  const double sensor_timestamp = sensor_points.stamp;
  double imu_stamp = imu_queue_.front().stamp;
  while (!imu_queue_.empty() || imu_stamp < sensor_timestamp) {
    imu_stamp = imu_queue_.front().stamp;
    measurement.imu_queue.emplace_back(imu_queue_.front());
    imu_queue_.pop_front();
  }
  measurement.lidar_points = sensor_points;
  points_queue_.pop_front();
  last_sensor_timestamp_ = sensor_timestamp;

  return true;
}

void ESKFLio::update_keyframe(
  const Eigen::Matrix4d & keyframe_pose, const PointCloudPtr & keyframe_points)
{
  submap::Submap submap(keyframe_pose, keyframe_points);
  submaps_.emplace_back(submap);

  if (keyframes_ == nullptr) {
    keyframes_.reset(new PointCloud);
  }

  PointType query_point;
  query_point.getVector3fMap() = keyframe_pose.block<3, 1>(0, 3).cast<float>();
  keyframes_->points.emplace_back(query_point);
  kd_tree_.setInputCloud(keyframes_);

  std::vector<int> indices(20);
  std::vector<float> distances(20);

  if (kd_tree_.nearestKSearch(query_point, 20, indices, distances)) {
    PointCloudPtr local_map(new PointCloud);
    for (std::size_t i = 0; i < indices.size(); i++) {
      *local_map += *submaps_[indices[i]].map_points;
    }
    registration_->setInputTarget(local_map);
  }
}

bool ESKFLio::is_keyframe_update(const Eigen::Matrix4d & candidate_keyframe_pose)
{
  if (submaps_.empty()) {
    return true;
  }

  const Eigen::Vector3d candidate_keyframe_p =
    candidate_keyframe_pose.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond candidate_keyframe_q(
    candidate_keyframe_pose.block<3, 3>(0, 0).cast<double>());

  double closest_p = std::numeric_limits<double>::infinity();
  int closest_idx = 0;
  int keyframe_idx = 0;
  int num_nearby = 0;
  for (auto & submap : submaps_) {
    const Eigen::Vector3d keyframe_p = submap.keyframe_pose.block<3, 1>(0, 3).cast<double>();
    const double delta_p = (candidate_keyframe_p - keyframe_p).norm();

    if (delta_p <= keyframe_update_distance_threshold_ * 1.5) {
      ++num_nearby;
    }

    if (delta_p < closest_p) {
      closest_p = delta_p;
      closest_idx = keyframe_idx;
    }

    keyframe_idx++;
  }

  Eigen::Vector3d closest_pose =
    submaps_[closest_idx].keyframe_pose.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond closest_q(
    submaps_[closest_idx].keyframe_pose.block<3, 3>(0, 0).cast<double>());

  const double dd = (candidate_keyframe_p - closest_pose).norm();

  Eigen::Quaterniond dq;

  if (candidate_keyframe_q.dot(closest_q) < 0.) {
    Eigen::Quaterniond lq = closest_q;
    lq.w() *= -1.;
    lq.x() *= -1.;
    lq.y() *= -1.;
    lq.z() *= -1.;
    dq = candidate_keyframe_q * lq.inverse();
  } else {
    dq = candidate_keyframe_q * closest_q.inverse();
  }

  double theta_rad = 2. * atan2(sqrt(pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2)), dq.w());
  double theta_deg = theta_rad * (180.0 / M_PI);

  bool is_updated_keyframe = false;
  if (std::fabs(dd) > keyframe_update_distance_threshold_ || std::fabs(theta_deg) > 45.0) {
    is_updated_keyframe = true;
  }

  if (std::fabs(dd) <= keyframe_update_distance_threshold_) {
    is_updated_keyframe = false;
  }

  if (
    std::fabs(dd) <= keyframe_update_distance_threshold_ &&
    std::fabs(theta_deg) > keyframe_update_yaw_threshold_ && num_nearby <= 1) {
    is_updated_keyframe = true;
  }

  return is_updated_keyframe;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ESKFLio::preprocessing(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  scan_voxel_grid_.setInputCloud(cloud_in);
  scan_voxel_grid_.filter(*filtered_cloud);

  crop_.setInputCloud(filtered_cloud);
  crop_.filter(*filtered_cloud);

  return filtered_cloud;
}

void ESKFLio::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_sensor)) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr base_to_sensor_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(
    *cloud, *base_to_sensor_points,
    lioamm_localizer_utils::convert_transform_to_matrix(base_to_sensor));

  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocess_points(new pcl::PointCloud<pcl::PointXYZI>);
  preprocess_points = preprocessing(base_to_sensor_points);

  sensor_type::PointCloud new_frame;
  new_frame.stamp = to_sec(msg->header);
  new_frame.raw_points = base_to_sensor_points;
  new_frame.preprocessing_points = preprocess_points;
  points_queue_.emplace_back(new_frame);
}

void ESKFLio::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped base_to_imu;
  if (!get_transform(base_frame_id_, msg->header.frame_id, base_to_imu)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get transform base_link to imu_link.");
    return;
  }

  Eigen::Transform<double, 3, Eigen::Affine> transform_matrix =
    lioamm_localizer_utils::get_eigen_transform(base_to_imu);

  sensor_type::Imu imu_data;
  imu_data.stamp = to_sec(msg->header);
  imu_data.linear_acceleration =
    transform_matrix *
    Eigen::Vector3d(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu_data.angular_velocity =
    transform_matrix *
    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  imu_queue_.emplace_back(imu_data);
}

bool ESKFLio::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transformation)
{
  try {
    transformation = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

void ESKFLio::publish_tf(
  const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
  const std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;

  broadcaster_->sendTransform(transform_stamped);
}

}  // namespace eskf_lio

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(eskf_lio::ESKFLio)
