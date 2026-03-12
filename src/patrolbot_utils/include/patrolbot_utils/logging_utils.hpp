#ifndef PATROLBOT_UTILS__LOGGING_UTILS_HPP_
#define PATROLBOT_UTILS__LOGGING_UTILS_HPP_

#include <string>

namespace patrolbot_utils
{

// Собирает короткое префиксированное сообщение для единообразного логирования.
std::string MakeLogMessage(const std::string & category, const std::string & message);

}  // namespace patrolbot_utils

#endif  // PATROLBOT_UTILS__LOGGING_UTILS_HPP_
