#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "patrolbot_mission_manager/mission_manager_node.hpp"

int main(int argc, char ** argv)
{
  // Как и в других runtime-пакетах PatrolBot, main остаётся минимальным.
  // Это упрощает сопровождение и не смешивает прикладную логику с bootstrap-кодом.
  rclcpp::init(argc, argv);

  // Вся сложная логика миссий находится внутри MissionManagerNode, а не здесь.
  auto node = std::make_shared<patrolbot_mission_manager::MissionManagerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
