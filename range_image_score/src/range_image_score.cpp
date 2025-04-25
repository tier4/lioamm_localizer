#include "range_image_score/range_image_score.hpp"

RangeImageScore::RangeImageScore() : Node("range_image_score")
{
  base_frame_id_ = this->declare_parameter("base_frame_id", "base_link");

  leaf_size_ = this->declare_parameter("leaf_size", 0.0);
  diff_threshold_ = this->declare_parameter("diff_threshold", 0.5);
  distance_threshold_ = this->declare_parameter("distance_threshold", 60.0);

  config_.fov = this->declare_parameter("fov", 0.0);
  config_.fov_up = this->declare_parameter("fov_up", 0.0);
  config_.fov_down = this->declare_parameter("fov_down", 0.0);
  config_.height = this->declare_parameter("height", 0);
  config_.width = this->declare_parameter("width", 0);

  dynamic_map_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
  dynamic_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 10);
  debug_sensor_image_publisher_ = image_transport::create_publisher(this, "debug_sensor_image");
  debug_map_image_publisher_ = image_transport::create_publisher(this, "debug_map_image");

  sensor_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&RangeImageScore::sensorCallback, this, std::placeholders::_1));
  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&RangeImageScore::mapCallback, this, std::placeholders::_1));
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", 5, std::bind(&RangeImageScore::poseCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::TransformStamped RangeImageScore::getTransform(
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf2_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return frame_transform;
}

void RangeImageScore::transformPointCloud(
  pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
  const geometry_msgs::msg::TransformStamped frame_transform)
{
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(frame_transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, frame_matrix);
}

void RangeImageScore::downsample(
  const pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
  const double leaf_size)
{
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_filter.setInputCloud(input_ptr);
  voxel_grid_filter.filter(*output_ptr);
}

Pixel2D RangeImageScore::getPixelId(const PointType point, double & range)
{
  Pixel2D pixel;
  pixel.x = -1;
  pixel.y = -1;
  range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  if (distance_threshold_ < range) return pixel;

  const double yaw = std::atan2(point.y, point.x);
  // const double pitch = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
  const double pitch = std::asin(point.z / range);

  int pixel_x = static_cast<int>(std::floor(config_.width * (0.5 * (1.0 - yaw / M_PI))));
  pixel_x = std::min(std::max(pixel_x, 0), config_.width - 1);

  // double pixel_y = std::floor(
  //   config_.height * (radian(config_.fov_up) - pitch) /
  //   (radian(config_.fov_up) - radian(config_.fov_down)));
  int pixel_y = static_cast<int>(std::floor(
    config_.height * (1.0 - ((pitch + std::fabs(radian(config_.fov_up))) / config_.fov))));
  pixel_y = std::min(std::max(pixel_y, 0), config_.height - 1);

  // int id = config_.width * pixel_y + pixel_x;
  pixel.x = pixel_x;
  pixel.y = pixel_y;
  return pixel;
}

pcl::PointCloud<PointType>::Ptr RangeImageScore::getNearestMapCloud(
  const geometry_msgs::msg::Pose pose)
{
  std::vector<int> indices;
  std::vector<float> distances;

  pcl::PointCloud<PointType>::Ptr nearest_cloud(new pcl::PointCloud<PointType>);

  PointType point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = pose.position.z;
  map_tree_->radiusSearch(point, distance_threshold_, indices, distances);

  for (const int idx : indices) {
    nearest_cloud->points.emplace_back(map_->points[idx]);
  }

  return nearest_cloud;
}

geometry_msgs::msg::PoseStamped RangeImageScore::getLatestPose(const rclcpp::Time stamp)
{
  geometry_msgs::msg::PoseStamped latest_pose;
  for (const auto & pose : pose_queue_) {
    const rclcpp::Time pose_time_stamp = latest_pose.header.stamp;
    if (stamp < pose_time_stamp) {
      break;
    }
    latest_pose = pose;
  }
  for (auto pose : pose_queue_) {
    rclcpp::Time pose_stamp = pose.header.stamp;
    if (pose_stamp < stamp) pose_queue_.pop_front();
  }
  return latest_pose;
}

void RangeImageScore::sensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr sensor_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *sensor_cloud);

  if (pose_queue_.empty() || sensor_cloud->points.empty() || map_tree_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "data is empty!");
    return;
  }

  // check latest time stamp
  geometry_msgs::msg::PoseStamped latest_pose = getLatestPose(msg->header.stamp);

  // map transform
  pcl::PointCloud<PointType>::Ptr map_transformed_cloud(new pcl::PointCloud<PointType>);
  transformPointCloud(
    getNearestMapCloud(latest_pose.pose), map_transformed_cloud,
    getTransform(base_frame_id_, latest_pose.header.frame_id));

  // std::map<int, double> map_image;
  // std::vector<Pixel2D> map_image;
  cv::Mat map_cv_image(config_.height, config_.width, CV_32FC1, cv::Scalar(-1));
  // map_image.resize(config_.width * config_.height);
  // for(std::size_t idx=0; idx < map_image.size();++idx)
  //   map_image[idx] = -1.0;
  pcl::PointCloud<PointType>::Ptr map_dynamic_cloud(new pcl::PointCloud<PointType>);
  for (const auto point : map_transformed_cloud->points) {
    double range = -1.0;
    const Pixel2D pixel = getPixelId(point, range);
    if (pixel.x == -1 or pixel.y == -1) continue;
    map_cv_image.at<float>(pixel.y, pixel.x) = range;
    if (map_cv_image.at<float>(pixel.y, pixel.x) < 0.0) {
      if (range < map_cv_image.at<float>(pixel.y, pixel.x)) {
        map_cv_image.at<float>(pixel.y, pixel.x) = range;
        map_dynamic_cloud->points.emplace_back(point);
        continue;
      }
    }
    map_cv_image.at<float>(pixel.y, pixel.x) = range;
    map_dynamic_cloud->points.emplace_back(point);
  }

  // sensor transform
  pcl::PointCloud<PointType>::Ptr sensor_transform_cloud(new pcl::PointCloud<PointType>);
  transformPointCloud(
    sensor_cloud, sensor_transform_cloud, getTransform(base_frame_id_, msg->header.frame_id));

  pcl::PointCloud<PointType>::Ptr dynamic_cloud(new pcl::PointCloud<PointType>);
  for (const auto point : sensor_transform_cloud->points) {
    double range = -1.0;
    const Pixel2D pixel = getPixelId(point, range);
    if (pixel.x == -1 or pixel.y == -1) continue;
    if (map_cv_image.at<float>(pixel.y, pixel.x) == -1.0) continue;
    if (map_cv_image.at<float>(pixel.y, pixel.x) < range) {
      const double diff = std::fabs(range - map_cv_image.at<float>(pixel.y, pixel.x));
      if (diff_threshold_ < diff) dynamic_cloud->points.emplace_back(point);
    }
  }

  pcl::PointCloud<PointType>::Ptr transformed_dynamic_cloud(new pcl::PointCloud<PointType>);
  transformPointCloud(
    dynamic_cloud, transformed_dynamic_cloud, getTransform(msg->header.frame_id, base_frame_id_));

  sensor_msgs::msg::PointCloud2 output_cloud_msg;
  pcl::toROSMsg(*transformed_dynamic_cloud, output_cloud_msg);
  output_cloud_msg.header = msg->header;
  dynamic_cloud_publisher_->publish(output_cloud_msg);

  sensor_msgs::msg::PointCloud2 output_map_cloud_msg;
  pcl::toROSMsg(*map_dynamic_cloud, output_map_cloud_msg);
  output_map_cloud_msg.header = msg->header;
  dynamic_map_cloud_publisher_->publish(output_map_cloud_msg);

  sensor_msgs::msg::Image::SharedPtr map_image_msgs =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, map_cv_image)
      .toImageMsg();
  map_image_msgs->header = msg->header;
  debug_map_image_publisher_.publish(map_image_msgs);
}

void RangeImageScore::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "map callback");

  if (!map_tree_) {
    map_tree_.reset(new pcl::search::KdTree<PointType>(true));
    score_tree_.reset(new pcl::search::KdTree<PointType>(true));
  }

  pcl::PointCloud<PointType>::Ptr map_raw(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *map_raw);
  map_.reset(new pcl::PointCloud<PointType>);

  if (leaf_size_ == 0.0)
    map_ = map_raw;
  else
    downsample(map_raw, map_, leaf_size_);

  map_tree_->setInputCloud(map_);
}

void RangeImageScore::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (map_->points.empty() || !map_tree_) {
    RCLCPP_WARN(get_logger(), "not initialize.");
    return;
  }

  pose_queue_.emplace_back(*msg);
}
