#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "patrolbot_base/base_node.hpp"

int main(int argc, char ** argv)
{
  // Точка входа intentionally минимальна: вся логика базы живёт в BaseNode,
  // а main отвечает только за жизненный цикл rclcpp.
  rclcpp::init(argc, argv);

  // Узел создаётся как shared_ptr, потому что rclcpp::spin работает именно с
  // таким владением объектом ноды.
  auto node = std::make_shared<patrolbot_base::BaseNode>();

  // spin передаёт управление в callback-модель ROS 2 и работает до остановки узла.
  rclcpp::spin(node);

  // Явный shutdown корректно завершает rclcpp-инфраструктуру и помогает
  // избежать неявных проблем при тестировании и интеграции.
  rclcpp::shutdown();
  return 0;
}
