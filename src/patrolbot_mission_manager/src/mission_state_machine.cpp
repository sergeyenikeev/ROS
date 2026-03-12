#include "patrolbot_mission_manager/mission_state_machine.hpp"

namespace patrolbot_mission_manager
{

std::string ToString(MissionState state)
{
  switch (state) {
    case MissionState::kIdle:
      return "IDLE";
    case MissionState::kValidatingRoute:
      return "VALIDATING_ROUTE";
    case MissionState::kWaitingForNav2:
      return "WAITING_FOR_NAV2";
    case MissionState::kNavigating:
      return "NAVIGATING";
    case MissionState::kRetryWait:
      return "RETRY_WAIT";
    case MissionState::kStopping:
      return "STOPPING";
    case MissionState::kCompleted:
      return "COMPLETED";
    case MissionState::kFailed:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

void MissionStateMachine::SetValidatingRoute()
{
  snapshot_ = MissionSnapshot{};
  snapshot_.state = MissionState::kValidatingRoute;
  snapshot_.active = true;
  mission_loaded_ = false;
}

TransitionDecision MissionStateMachine::ActivateMission(
  const patrolbot_utils::MissionConfig & mission,
  bool nav2_ready)
{
  mission_ = mission;
  mission_loaded_ = true;

  snapshot_ = MissionSnapshot{};
  snapshot_.state = nav2_ready ? MissionState::kNavigating : MissionState::kWaitingForNav2;
  snapshot_.active = true;

  return {
    nav2_ready ? MissionCommand::kDispatchGoal : MissionCommand::kWaitForNav2,
    nav2_ready ? "миссия активирована и готова к отправке первой цели" :
    "миссия активирована, ожидание action-сервера Nav2"};
}

TransitionDecision MissionStateMachine::HandleNav2Ready()
{
  if (snapshot_.state != MissionState::kWaitingForNav2 || !snapshot_.active) {
    return {};
  }

  snapshot_.state = MissionState::kNavigating;
  return {MissionCommand::kDispatchGoal, "action-сервер Nav2 доступен"};
}

TransitionDecision MissionStateMachine::HandleGoalSuccess()
{
  if (!mission_loaded_ || !snapshot_.active) {
    return {};
  }

  snapshot_.completed_waypoints += 1U;
  snapshot_.retry_count_for_current_waypoint = 0U;
  snapshot_.last_error.clear();

  if (snapshot_.current_waypoint_index + 1U < mission_.waypoints.size()) {
    snapshot_.current_waypoint_index += 1U;
    snapshot_.state = MissionState::kNavigating;
    return {MissionCommand::kDispatchGoal, "переход к следующей точке маршрута"};
  }

  if (mission_.loop_forever) {
    snapshot_.loop_count += 1U;
    snapshot_.current_waypoint_index = 0U;
    snapshot_.state = MissionState::kNavigating;
    return {MissionCommand::kDispatchGoal, "маршрут завершён, начинается новый цикл"};
  }

  snapshot_.state = MissionState::kCompleted;
  snapshot_.active = false;
  return {MissionCommand::kCompleteMission, "маршрут успешно завершён"};
}

TransitionDecision MissionStateMachine::HandleGoalFailure(const std::string & reason)
{
  return HandleGoalError(reason);
}

TransitionDecision MissionStateMachine::HandleGoalTimeout()
{
  return HandleGoalError("превышен таймаут прохождения точки");
}

TransitionDecision MissionStateMachine::HandleRetryTimer()
{
  if (snapshot_.state != MissionState::kRetryWait || !snapshot_.active) {
    return {};
  }

  snapshot_.state = MissionState::kNavigating;
  return {MissionCommand::kDispatchGoal, "выполняется повторная отправка цели"};
}

TransitionDecision MissionStateMachine::RequestStop(const std::string & reason)
{
  snapshot_.last_error = reason;

  if (snapshot_.state == MissionState::kNavigating) {
    snapshot_.state = MissionState::kStopping;
    snapshot_.active = false;
    return {MissionCommand::kCancelGoal, reason};
  }

  snapshot_.state = MissionState::kIdle;
  snapshot_.active = false;
  return {MissionCommand::kStopped, reason};
}

TransitionDecision MissionStateMachine::FinishStop(const std::string & reason)
{
  snapshot_.state = MissionState::kIdle;
  snapshot_.active = false;
  snapshot_.last_error = reason;
  return {MissionCommand::kStopped, reason};
}

TransitionDecision MissionStateMachine::AbortMission(const std::string & reason)
{
  snapshot_.state = MissionState::kFailed;
  snapshot_.active = false;
  snapshot_.last_error = reason;
  return {MissionCommand::kFailMission, reason};
}

const patrolbot_utils::MissionConfig * MissionStateMachine::mission() const
{
  return mission_loaded_ ? &mission_ : nullptr;
}

const patrolbot_utils::WaypointConfig * MissionStateMachine::CurrentWaypoint() const
{
  if (!mission_loaded_ || snapshot_.current_waypoint_index >= mission_.waypoints.size()) {
    return nullptr;
  }

  return &mission_.waypoints[snapshot_.current_waypoint_index];
}

const MissionSnapshot & MissionStateMachine::snapshot() const
{
  return snapshot_;
}

TransitionDecision MissionStateMachine::HandleGoalError(const std::string & reason)
{
  if (!mission_loaded_) {
    return {};
  }

  snapshot_.last_error = reason;
  const auto * waypoint = CurrentWaypoint();
  if (waypoint == nullptr) {
    snapshot_.state = MissionState::kFailed;
    snapshot_.active = false;
    return {MissionCommand::kFailMission, "текущая точка маршрута недоступна"};
  }

  if (static_cast<int>(snapshot_.retry_count_for_current_waypoint) < waypoint->retries) {
    snapshot_.retry_count_for_current_waypoint += 1U;
    snapshot_.state = MissionState::kRetryWait;
    return {MissionCommand::kScheduleRetry, reason};
  }

  snapshot_.failed_waypoints += 1U;
  snapshot_.retry_count_for_current_waypoint = 0U;

  if (mission_.continue_on_error) {
    if (snapshot_.current_waypoint_index + 1U < mission_.waypoints.size()) {
      snapshot_.current_waypoint_index += 1U;
      snapshot_.state = MissionState::kNavigating;
      return {MissionCommand::kDispatchGoal, "ошибка зафиксирована, переход к следующей точке"};
    }

    if (mission_.loop_forever) {
      snapshot_.loop_count += 1U;
      snapshot_.current_waypoint_index = 0U;
      snapshot_.state = MissionState::kNavigating;
      return {MissionCommand::kDispatchGoal, "ошибка зафиксирована, начинается новый цикл"};
    }

    snapshot_.state = MissionState::kCompleted;
    snapshot_.active = false;
    return {MissionCommand::kCompleteMission, "маршрут завершён с допустимыми ошибками"};
  }

  snapshot_.state = MissionState::kFailed;
  snapshot_.active = false;
  return {MissionCommand::kFailMission, reason};
}

}  // namespace patrolbot_mission_manager
