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
  // Для обязательных полей ошибка формируется сразу и с точным именем ключа,
  // чтобы оператор быстро находил проблему в YAML.
  if (!node[key]) {
    throw std::runtime_error("в YAML отсутствует обязательное поле '" + key + "'");
  }
  return node[key].as<std::string>();
}

double ReadRequiredDouble(const YAML::Node & node, const std::string & key)
{
  // Отдельный helper нужен, чтобы поведение для обязательных числовых полей
  // было одинаковым во всех частях маршрута.
  if (!node[key]) {
    throw std::runtime_error("в YAML отсутствует обязательное поле '" + key + "'");
  }
  return node[key].as<double>();
}

}  // namespace

MissionConfig LoadMissionConfig(const std::string & path)
{
  // Явная проверка пути позволяет выдавать понятную ошибку ещё до чтения YAML.
  if (path.empty()) {
    throw std::runtime_error("путь к YAML-файлу миссии пуст");
  }

  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("YAML-файл миссии не найден: " + path);
  }

  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node mission_node = root["mission"];
  // Наличие корневой секции mission обязательно: это защищает от случайной
  // подстановки другого YAML вместо конфигурации патруля.
  if (!mission_node) {
    throw std::runtime_error("в YAML отсутствует секция 'mission'");
  }

  MissionConfig mission;
  // Общие параметры миссии читаются с разумными значениями по умолчанию, чтобы
  // минимальный маршрут оставался компактным и удобным для правки.
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
  // Сначала валидируются общие параметры миссии, а затем каждая точка маршрута.
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
      // Имена и координаты каждой точки обязательны, потому что именно на них
      // завязаны постановка цели и человекочитаемые логи миссии.
      waypoint.name = ReadRequiredString(waypoint_node, "name");
      waypoint.x = ReadRequiredDouble(waypoint_node, "x");
      waypoint.y = ReadRequiredDouble(waypoint_node, "y");
      const double yaw_deg = ReadRequiredDouble(waypoint_node, "yaw_deg");
      // В YAML угол удобнее задавать в градусах, но внутри проекта везде
      // используется радианная мера для совместимости с tf2.
      waypoint.yaw_rad = yaw_deg * kPi / 180.0;
      waypoint.timeout_sec =
        waypoint_node["timeout_sec"] ?
        waypoint_node["timeout_sec"].as<double>() :
        mission.default_timeout_sec;
      waypoint.retries =
        waypoint_node["retries"] ? waypoint_node["retries"].as<int>() : mission.default_retries;

      // Валидация waypoint выполняется с указанием индекса, чтобы маршрут из
      // десятков точек можно было быстро исправлять без лишнего поиска.
      RequireNotEmpty("waypoint[" + std::to_string(index) + "].name", waypoint.name, &issues);
      RequirePositive(
        "waypoint[" + std::to_string(index) + "].timeout_sec",
        waypoint.timeout_sec,
        &issues);
      if (waypoint.retries < 0) {
        issues.push_back(
          {"waypoint[" + std::to_string(index) + "].retries", "не должно быть отрицательным"});
      }

      // Даже при ошибках продолжаем разбор всего файла, чтобы вернуть полный
      // набор проблем за один проход CI или ручной проверки.
      mission.waypoints.push_back(waypoint);
    }
  }

  // Итоговый отчёт по ошибкам поднимается одним исключением, пригодным и для
  // логов узлов, и для unit/integration тестов.
  if (HasIssues(issues)) {
    throw std::runtime_error(FormatIssues(issues));
  }

  return mission;
}

}  // namespace patrolbot_utils
