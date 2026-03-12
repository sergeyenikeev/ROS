#include "patrolbot_utils/route_loader.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

#include "patrolbot_utils/parameter_validation.hpp"

namespace patrolbot_utils
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string ReadRequiredString(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    throw std::runtime_error("в YAML отсутствует обязательное поле '" + key + "'");
  }
  return node[key].as<std::string>();
}

double ReadRequiredDouble(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    throw std::runtime_error("в YAML отсутствует обязательное поле '" + key + "'");
  }
  return node[key].as<double>();
}

}  // namespace

MissionConfig LoadMissionConfig(const std::string & path)
{
  if (path.empty()) {
    throw std::runtime_error("путь к YAML-файлу миссии пуст");
  }

  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("YAML-файл миссии не найден: " + path);
  }

  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node mission_node = root["mission"];
  if (!mission_node) {
    throw std::runtime_error("в YAML отсутствует секция 'mission'");
  }

  MissionConfig mission;
  mission.name = ReadRequiredString(mission_node, "name");
  mission.frame_id = mission_node["frame_id"] ? mission_node["frame_id"].as<std::string>() : "map";
  mission.autostart = mission_node["autostart"] ? mission_node["autostart"].as<bool>() : false;
  mission.loop_forever =
    mission_node["loop_forever"] ? mission_node["loop_forever"].as<bool>() : false;
  mission.continue_on_error =
    mission_node["continue_on_error"] ? mission_node["continue_on_error"].as<bool>() : false;
  mission.default_timeout_sec =
    mission_node["default_timeout_sec"] ? mission_node["default_timeout_sec"].as<double>() : 90.0;
  mission.default_retries =
    mission_node["default_retries"] ? mission_node["default_retries"].as<int>() : 2;
  mission.retry_backoff_sec =
    mission_node["retry_backoff_sec"] ? mission_node["retry_backoff_sec"].as<double>() : 5.0;

  ValidationIssues issues;
  RequireNotEmpty("mission.name", mission.name, &issues);
  RequireNotEmpty("mission.frame_id", mission.frame_id, &issues);
  RequirePositive("mission.default_timeout_sec", mission.default_timeout_sec, &issues);
  RequirePositive("mission.retry_backoff_sec", mission.retry_backoff_sec, &issues);

  if (mission.default_retries < 0) {
    issues.push_back({"mission.default_retries", "не должно быть отрицательным"});
  }

  const YAML::Node waypoints_node = mission_node["waypoints"];
  if (!waypoints_node || !waypoints_node.IsSequence() || waypoints_node.size() == 0) {
    issues.push_back({"mission.waypoints", "должен содержать хотя бы одну точку"});
  } else {
    for (std::size_t index = 0; index < waypoints_node.size(); ++index) {
      const YAML::Node waypoint_node = waypoints_node[index];
      WaypointConfig waypoint;
      waypoint.name = ReadRequiredString(waypoint_node, "name");
      waypoint.x = ReadRequiredDouble(waypoint_node, "x");
      waypoint.y = ReadRequiredDouble(waypoint_node, "y");
      const double yaw_deg = ReadRequiredDouble(waypoint_node, "yaw_deg");
      waypoint.yaw_rad = yaw_deg * kPi / 180.0;
      waypoint.timeout_sec =
        waypoint_node["timeout_sec"] ?
        waypoint_node["timeout_sec"].as<double>() :
        mission.default_timeout_sec;
      waypoint.retries =
        waypoint_node["retries"] ? waypoint_node["retries"].as<int>() : mission.default_retries;

      RequireNotEmpty("waypoint[" + std::to_string(index) + "].name", waypoint.name, &issues);
      RequirePositive(
        "waypoint[" + std::to_string(index) + "].timeout_sec",
        waypoint.timeout_sec,
        &issues);
      if (waypoint.retries < 0) {
        issues.push_back(
          {"waypoint[" + std::to_string(index) + "].retries", "не должно быть отрицательным"});
      }

      mission.waypoints.push_back(waypoint);
    }
  }

  if (HasIssues(issues)) {
    throw std::runtime_error(FormatIssues(issues));
  }

  return mission;
}

}  // namespace patrolbot_utils
