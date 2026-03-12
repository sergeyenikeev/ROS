#ifndef PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
#define PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_

#include <cstddef>
#include <string>

#include "patrolbot_utils/route_loader.hpp"

namespace patrolbot_mission_manager
{

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

struct TransitionDecision
{
  MissionCommand command{MissionCommand::kNone};
  std::string reason;
};

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

// Отдельная state machine позволяет тестировать переходы без запуска ROS 2 action-клиента.
class MissionStateMachine
{
public:
  void SetValidatingRoute();
  TransitionDecision ActivateMission(
    const patrolbot_utils::MissionConfig & mission,
    bool nav2_ready);
  TransitionDecision HandleNav2Ready();
  TransitionDecision HandleGoalSuccess();
  TransitionDecision HandleGoalFailure(const std::string & reason);
  TransitionDecision HandleGoalTimeout();
  TransitionDecision HandleRetryTimer();
  TransitionDecision RequestStop(const std::string & reason);
  TransitionDecision FinishStop(const std::string & reason);
  TransitionDecision AbortMission(const std::string & reason);

  const patrolbot_utils::MissionConfig * mission() const;
  const patrolbot_utils::WaypointConfig * CurrentWaypoint() const;
  const MissionSnapshot & snapshot() const;

private:
  TransitionDecision HandleGoalError(const std::string & reason);

  patrolbot_utils::MissionConfig mission_;
  MissionSnapshot snapshot_;
  bool mission_loaded_{false};
};

}  // namespace patrolbot_mission_manager

#endif  // PATROLBOT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
