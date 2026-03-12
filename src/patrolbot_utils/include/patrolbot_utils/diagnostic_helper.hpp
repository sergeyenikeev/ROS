#ifndef PATROLBOT_UTILS__DIAGNOSTIC_HELPER_HPP_
#define PATROLBOT_UTILS__DIAGNOSTIC_HELPER_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace patrolbot_utils
{

using DiagnosticKeyValues = std::vector<std::pair<std::string, std::string>>;

// Формирует сообщение диагностики в одном месте, чтобы все узлы использовали одинаковый стиль.
diagnostic_msgs::msg::DiagnosticStatus MakeDiagnosticStatus(
  const std::string & name,
  std::uint8_t level,
  const std::string & message,
  const DiagnosticKeyValues & values);

}  // namespace patrolbot_utils

#endif  // PATROLBOT_UTILS__DIAGNOSTIC_HELPER_HPP_
