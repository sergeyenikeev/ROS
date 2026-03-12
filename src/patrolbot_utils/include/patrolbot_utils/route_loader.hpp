#ifndef PATROLBOT_UTILS__ROUTE_LOADER_HPP_
#define PATROLBOT_UTILS__ROUTE_LOADER_HPP_

#include <string>
#include <vector>

namespace patrolbot_utils
{

// Описание одной точки маршрута уже в удобном для mission manager виде.
struct WaypointConfig
{
  // Человекочитаемое имя точки для логов и статуса миссии.
  std::string name;

  // Координаты цели в системе frame_id миссии.
  double x{0.0};
  double y{0.0};

  // Ориентация точки в радианах. В YAML она задаётся в градусах и переводится при загрузке.
  double yaw_rad{0.0};

  // Индивидуальный таймаут прохождения waypoint.
  double timeout_sec{0.0};

  // Сколько повторных попыток допускается именно для этой точки.
  int retries{0};
};

// Полная миссия патрулирования, объединяющая общие параметры и список waypoint-ов.
struct MissionConfig
{
  // Имя миссии отображается в статусе и логах.
  std::string name;

  // Система координат, в которой заданы waypoint-ы.
  std::string frame_id{"map"};

  // Нужно ли стартовать миссию автоматически после запуска узла.
  bool autostart{false};

  // Нужно ли бесконечно повторять маршрут по кругу.
  bool loop_forever{false};

  // Разрешено ли продолжать маршрут после невосстановимой ошибки отдельной точки.
  bool continue_on_error{false};

  // Значения по умолчанию, применяемые к waypoint-ам, где параметры не заданы явно.
  double default_timeout_sec{90.0};
  int default_retries{2};
  double retry_backoff_sec{5.0};

  // Список точек маршрута в порядке обхода.
  std::vector<WaypointConfig> waypoints;
};

// Загружает YAML-файл миссии и возвращает уже провалидированную конфигурацию.
MissionConfig LoadMissionConfig(const std::string & path);

}  // namespace patrolbot_utils

#endif  // PATROLBOT_UTILS__ROUTE_LOADER_HPP_
