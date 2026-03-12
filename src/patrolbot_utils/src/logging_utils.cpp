#include "patrolbot_utils/logging_utils.hpp"

namespace patrolbot_utils
{

std::string MakeLogMessage(const std::string & category, const std::string & message)
{
  return "[" + category + "] " + message;
}

}  // namespace patrolbot_utils
