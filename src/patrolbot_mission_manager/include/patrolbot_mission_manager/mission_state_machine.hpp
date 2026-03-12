#ifndef PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
#define PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_

#include <cstddef>
#include <string>

#include "patrolbot_utils/route_loader.hpp"

namespace patrolbot_mission_manager
{

// Состояния конечного автомата mission manager.
enum class MissionState
{
  kIdle,
  kValidatingRoute,
  kWaitingForNav2,
  kNavigating,
  kRetryWait,
  kStopping,
  kCompleted,
  kFailed,
};

// Команда описывает, какое внешнее действие должен сделать узел
// после очередного перехода state machine.
enum class MissionCommand
{
  kNone,
  kWaitForNav2,
  kDispatchGoal,
  kScheduleRetry,
  kCancelGoal,
  kCompleteMission,
  kFailMission,
  kStopped,
};

// В одном объекте возвращаем и новое "намерение" автомата, и текстовую причину,
// пригодную для логов, тестов и публикации статуса.
struct TransitionDecision
{
  MissionCommand command{MissionCommand::kNone};
  std::string reason;
};

// Snapshot хранит текущее внешне наблюдаемое состояние миссии.
// Он используется и в статусном топике, и в unit-тестах, и в логике узла.
struct MissionSnapshot
{
  MissionState state{MissionState::kIdle};
  bool active{false};
  std::size_t current_waypoint_index{0};
  std::size_t completed_waypoints{0};
  std::size_t failed_waypoints{0};
  std::size_t loop_count{0};
  std::size_t retry_count_for_current_waypoint{0};
  std::string last_error;
};

// Преобразует состояние в строку для логов, статуса и документации.
std::string ToString(MissionState state);

// Отдельная state machine позволяет тестировать правила переходов без запуска ROS 2 action-клиента.
// Это намеренно "чистая" логика: она не знает о таймерах ROS, сервисах и action transport.
class MissionStateMachine
{
public:
  // Вызывается перед валидацией нового маршрута.
  void SetValidatingRoute();

  // Активирует новую миссию и сразу решает, можно ли стартовать навигацию прямо сейчас.
  TransitionDecision ActivateMission(
    const patrolbot_utils::MissionConfig & mission,
    bool nav2_ready);

  // Сигнал о появлении Nav2 после состояния ожидания.
  TransitionDecision HandleNav2Ready();

  // Целевая точка достигнута.
  TransitionDecision HandleGoalSuccess();

  // Nav2 вернул ошибку при прохождении текущей точки.
  TransitionDecision HandleGoalFailure(const std::string & reason);

  // Текущая точка не была достигнута за допустимое время.
  TransitionDecision HandleGoalTimeout();

  // Вызывается после истечения backoff-задержки перед новой попыткой.
  TransitionDecision HandleRetryTimer();

  // Оператор или внешний код запросил остановку миссии.
  TransitionDecision RequestStop(const std::string & reason);

  // Используется после подтверждённой отмены текущей action-цели.
  TransitionDecision FinishStop(const std::string & reason);

  // Используется для немедленного аварийного завершения миссии.
  TransitionDecision AbortMission(const std::string & reason);

  // Даёт доступ к текущей миссии, если она уже загружена и активирована.
  const patrolbot_utils::MissionConfig * mission() const;

  // Возвращает текущую точку маршрута или nullptr, если состояние неконсистентно.
  const patrolbot_utils::WaypointConfig * CurrentWaypoint() const;

  // Возвращает снимок состояния, пригодный для публикации наружу.
  const MissionSnapshot & snapshot() const;

private:
  // Общая логика обработки ошибки цели: повторы, переход к следующей точке
  // или аварийное завершение миссии.
  TransitionDecision HandleGoalError(const std::string & reason);

  patrolbot_utils::MissionConfig mission_;
  MissionSnapshot snapshot_;
  bool mission_loaded_{false};
};

}  // namespace patrolbot_mission_manager

#endif  // PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
