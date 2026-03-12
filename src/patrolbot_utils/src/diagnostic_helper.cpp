#include "patrolbot_utils/diagnostic_helper.hpp"

#include "diagnostic_msgs/msg/key_value.hpp"

namespace patrolbot_utils
{

diagnostic_msgs::msg::DiagnosticStatus MakeDiagnosticStatus(
  const std::string & name,
  std::uint8_t level,
  const std::string & message,
  const DiagnosticKeyValues & values)
{
  // Helper даёт всем узлам один и тот же шаблон diagnostic status, чтобы
  // агрегаторы и операторские скрипты не зависели от частных реализаций.
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "patrolbot";
  status.level = level;
  status.message = message;

  // Каждая пара key/value переносится в стандартный формат ROS diagnostics.
  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  return status;
}

}  // namespace patrolbot_utils
