#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "patrolbot_base/base_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<patrolbot_base::BaseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
