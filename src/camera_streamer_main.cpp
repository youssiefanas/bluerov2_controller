#include "bluerov2_controller/camera_streamer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bluerov2_controller::CameraStreamerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
