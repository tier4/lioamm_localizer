#include "range_image_score/range_image_score.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto range_image_score = std::make_shared<RangeImageScore>();
  rclcpp::spin(range_image_score);
  rclcpp::shutdown();
  return 0;
}
