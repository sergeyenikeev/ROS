#include "patrolbot_utils/logging_utils.hpp"

namespace patrolbot_utils
{

std::string MakeLogMessage(const std::string & category, const std::string & message)
{
  // Единый префикс упрощает фильтрацию логов по смысловым категориям при
  // разборе длинных запусков bringup, Nav2 и mission manager.
  return "[" + category + "] " + message;
}

}  // namespace patrolbot_utils
