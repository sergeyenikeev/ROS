#include "patrolbot_mission_manager/mission_state_machine.hpp"

namespace patrolbot_mission_manager
{

// Строковое представление используется в логах, статусном сообщении и тестах,
// поэтому оно должно оставаться стабильным и понятным оператору.
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
  // Новый цикл запуска всегда стартует с чистого снимка состояния, иначе
  // статистика предыдущей миссии могла бы протечь в новый проход.
  snapshot_ = MissionSnapshot{};
  snapshot_.state = MissionState::kValidatingRoute;
  snapshot_.active = true;
  mission_loaded_ = false;
}

TransitionDecision MissionStateMachine::ActivateMission(
  const patrolbot_utils::MissionConfig & mission,
  bool nav2_ready)
{
  // При активации маршрута state machine получает уже провалидированную
  // конфигурацию и дальше работает только с ней.
  mission_ = mission;
  mission_loaded_ = true;

  snapshot_ = MissionSnapshot{};
  snapshot_.state = nav2_ready ? MissionState::kNavigating : MissionState::kWaitingForNav2;
  snapshot_.active = true;

  // Если Nav2 готов, можно сразу отправлять первую цель. Иначе узел перейдёт
  // в состояние ожидания внешней готовности action-сервера.
  return {
    nav2_ready ? MissionCommand::kDispatchGoal : MissionCommand::kWaitForNav2,
    nav2_ready ? "миссия активирована и готова к отправке первой цели" :
    "миссия активирована, ожидание action-сервера Nav2"};
}

TransitionDecision MissionStateMachine::HandleNav2Ready()
{
  // Сигнал о готовности Nav2 имеет смысл только в состоянии ожидания.
  if (snapshot_.state != MissionState::kWaitingForNav2 || !snapshot_.active) {
    return {};
  }

  snapshot_.state = MissionState::kNavigating;
  return {MissionCommand::kDispatchGoal, "action-сервер Nav2 доступен"};
}

TransitionDecision MissionStateMachine::HandleGoalSuccess()
{
  // Успешное прохождение точки фиксирует прогресс миссии и сбрасывает счётчик
  // повторов текущей waypoint.
  if (!mission_loaded_ || !snapshot_.active) {
    return {};
  }

  snapshot_.completed_waypoints += 1U;
  snapshot_.retry_count_for_current_waypoint = 0U;
  snapshot_.last_error.clear();

  // Если в маршруте остались точки, автомат просто сдвигает индекс и просит
  // внешнюю обвязку отправить следующую цель.
  if (snapshot_.current_waypoint_index + 1U < mission_.waypoints.size()) {
    snapshot_.current_waypoint_index += 1U;
    snapshot_.state = MissionState::kNavigating;
    return {MissionCommand::kDispatchGoal, "переход к следующей точке маршрута"};
  }

  // В циклическом режиме последняя точка не завершает миссию, а запускает новый
  // обход маршрута с увеличением счётчика циклов.
  if (mission_.loop_forever) {
    snapshot_.loop_count += 1U;
    snapshot_.current_waypoint_index = 0U;
    snapshot_.state = MissionState::kNavigating;
    return {MissionCommand::kDispatchGoal, "маршрут завершён, начинается новый цикл"};
  }

  // В одноразовом сценарии последняя точка завершает миссию штатным успехом.
  snapshot_.state = MissionState::kCompleted;
  snapshot_.active = false;
  return {MissionCommand::kCompleteMission, "маршрут успешно завершён"};
}

TransitionDecision MissionStateMachine::HandleGoalFailure(const std::string & reason)
{
  // Отказ Nav2 и прочие ошибки цели подчиняются общей политике повторов.
  return HandleGoalError(reason);
}

TransitionDecision MissionStateMachine::HandleGoalTimeout()
{
  // Таймаут обрабатывается как частный случай ошибки цели с отдельным текстом.
  return HandleGoalError("превышен таймаут прохождения точки");
}

TransitionDecision MissionStateMachine::HandleRetryTimer()
{
  // Повтор корректен только в состоянии ожидания между попытками.
  if (snapshot_.state != MissionState::kRetryWait || !snapshot_.active) {
    return {};
  }

  snapshot_.state = MissionState::kNavigating;
  return {MissionCommand::kDispatchGoal, "выполняется повторная отправка цели"};
}

TransitionDecision MissionStateMachine::RequestStop(const std::string & reason)
{
  // Последняя причина остановки сохраняется в снимке состояния для статуса.
  snapshot_.last_error = reason;

  // Если сейчас выполняется активная навигация, требуется сначала отменить goal.
  if (snapshot_.state == MissionState::kNavigating) {
    snapshot_.state = MissionState::kStopping;
    snapshot_.active = false;
    return {MissionCommand::kCancelGoal, reason};
  }

  // В остальных состояниях отдельной активной цели уже нет, поэтому остановка
  // завершается немедленно.
  snapshot_.state = MissionState::kIdle;
  snapshot_.active = false;
  return {MissionCommand::kStopped, reason};
}

TransitionDecision MissionStateMachine::FinishStop(const std::string & reason)
{
  // Этот переход вызывается после подтверждённой отмены action-цели Nav2.
  snapshot_.state = MissionState::kIdle;
  snapshot_.active = false;
  snapshot_.last_error = reason;
  return {MissionCommand::kStopped, reason};
}

TransitionDecision MissionStateMachine::AbortMission(const std::string & reason)
{
  // Аварийное завершение применяется для фатальных состояний, после которых
  // продолжать миссию уже небезопасно.
  snapshot_.state = MissionState::kFailed;
  snapshot_.active = false;
  snapshot_.last_error = reason;
  return {MissionCommand::kFailMission, reason};
}

const patrolbot_utils::MissionConfig * MissionStateMachine::mission() const
{
  // Наружу возвращается либо активная миссия, либо nullptr как явный сигнал,
  // что маршрут сейчас недоступен.
  return mission_loaded_ ? &mission_ : nullptr;
}

const patrolbot_utils::WaypointConfig * MissionStateMachine::CurrentWaypoint() const
{
  // Защита от выхода за границы нужна на случай рассинхронизации внешней обвязки.
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
  // Без загруженной миссии невозможно корректно определить политику повторов.
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

  // Пока лимит повторов не исчерпан, автомат не валит миссию, а уходит
  // в состояние ожидания backoff-паузы перед повторной отправкой цели.
  if (static_cast<int>(snapshot_.retry_count_for_current_waypoint) < waypoint->retries) {
    snapshot_.retry_count_for_current_waypoint += 1U;
    snapshot_.state = MissionState::kRetryWait;
    return {MissionCommand::kScheduleRetry, reason};
  }

  // После исчерпания повторов точка считается окончательно неуспешной.
  snapshot_.failed_waypoints += 1U;
  snapshot_.retry_count_for_current_waypoint = 0U;

  // Политика continue_on_error позволяет пройти оставшийся маршрут, даже если
  // одна из точек оказалась недостижимой.
  if (mission_.continue_on_error) {
    if (snapshot_.current_waypoint_index + 1U < mission_.waypoints.size()) {
      snapshot_.current_waypoint_index += 1U;
      snapshot_.state = MissionState::kNavigating;
      return {MissionCommand::kDispatchGoal, "ошибка зафиксирована, переход к следующей точке"};
    }

    // В циклическом режиме после ошибки на последней точке допускается новый
    // проход, если это разрешено политикой миссии.
    if (mission_.loop_forever) {
      snapshot_.loop_count += 1U;
      snapshot_.current_waypoint_index = 0U;
      snapshot_.state = MissionState::kNavigating;
      return {MissionCommand::kDispatchGoal, "ошибка зафиксирована, начинается новый цикл"};
    }

    // Если маршрут закончился и ошибки допустимы, миссия считается завершённой
    // с нефатальными сбоями.
    snapshot_.state = MissionState::kCompleted;
    snapshot_.active = false;
    return {MissionCommand::kCompleteMission, "маршрут завершён с допустимыми ошибками"};
  }

  snapshot_.state = MissionState::kFailed;
  snapshot_.active = false;
  return {MissionCommand::kFailMission, reason};
}

}  // namespace patrolbot_mission_manager
