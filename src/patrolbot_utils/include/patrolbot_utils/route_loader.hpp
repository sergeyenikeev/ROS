#ifndef PATROLBOT_UTILS__ROUTE_LOADER_HPP_
#define PATROLBOT_UTILS__ROUTE_LOADER_HPP_

#include <string>
#include <vector>

namespace patrolbot_utils
{

struct WaypointConfig
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
  double timeout_sec{0.0};
  int retries{0};
};

struct MissionConfig
{
  std::string name;
  std::string frame_id{"map"};
  bool autostart{false};
  bool loop_forever{false};
  bool continue_on_error{false};
  double default_timeout_sec{90.0};
  int default_retries{2};
  double retry_backoff_sec{5.0};
  std::vector<WaypointConfig> waypoints;
};

// Загружает YAML-файл миссии и возвращает уже провалидированную конфигурацию.
MissionConfig LoadMissionConfig(const std::string & path);

}  // namespace patrolbot_utils

#endif  // PATROLBOT_UTILS__ROUTE_LOADER_HPP_
