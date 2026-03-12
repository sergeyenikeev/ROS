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
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "patrolbot";
  status.level = level;
  status.message = message;

  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  return status;
}

}  // namespace patrolbot_utils
