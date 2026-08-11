#include <rclcpp/rclcpp.hpp>

#include "robot_dog_gait/keyboard_teleop_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<robot_dog_gait::KeyboardTeleopNode>();
    node->run();
  }
  rclcpp::shutdown();
  return 0;
}
