#include "bluerov2_controller/rov_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bluerov2_controller::RovControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
