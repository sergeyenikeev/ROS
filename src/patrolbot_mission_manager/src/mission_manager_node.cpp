#include "patrolbot_mission_manager/mission_manager_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "patrolbot_utils/logging_utils.hpp"
#include "patrolbot_utils/route_loader.hpp"

namespace patrolbot_mission_manager
{

// Конструктор поднимает все компоненты, связывающие конечный автомат миссии
// с инфраструктурой ROS 2: параметры, сервисы, action-клиент, публикацию
// статуса и периодический heartbeat-таймер.
MissionManagerNode::MissionManagerNode()
: Node("patrolbot_mission_manager")
{
  LoadParameters();

  // Статус публикуется отдельным сообщением, чтобы внешние утилиты, launch-тесты
  // и операторские сценарии могли наблюдать миссию без прямого доступа к
  // внутренним полям узла.
  status_publisher_ =
    create_publisher<patrolbot_interfaces::msg::PatrolStatus>(status_topic_, 10);
  start_service_ = create_service<patrolbot_interfaces::srv::StartPatrol>(
    "/patrol/start",
    std::bind(
      &MissionManagerNode::StartServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  stop_service_ = create_service<patrolbot_interfaces::srv::StopPatrol>(
    "/patrol/stop",
    std::bind(
      &MissionManagerNode::StopServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);

  // Таймер статуса выступает в роли лёгкого главного цикла mission manager:
  // он публикует снимок состояния, выполняет отложенный автозапуск и следит
  // за появлением action-сервера Nav2.
  const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_publish_rate_hz_));
  status_timer_ = create_wall_timer(
    status_period,
    std::bind(&MissionManagerNode::StatusTimerCallback, this));

  std::string error_text;
  // Предварительная загрузка маршрута позволяет показать ошибку конфигурации
  // сразу после старта узла, а не только после первого вызова сервиса start.
  mission_config_loaded_ = ReloadMissionConfiguration(&error_text);
  if (mission_config_loaded_) {
    autostart_pending_ = mission_config_.autostart;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "%s",
      patrolbot_utils::MakeLogMessage(
        "ошибка", "не удалось предварительно загрузить миссию: " + error_text).c_str());
  }

  PublishStatus();

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "узел",
      "mission_manager запущен, mission_file=" + mission_file_).c_str());
}

void MissionManagerNode::LoadParameters()
{
  std::string default_mission_file;
  try {
    // Предпочтительный путь для установленного пакета в полноценной ROS 2 среде.
    default_mission_file =
      ament_index_cpp::get_package_share_directory("patrolbot_mission_manager") +
      "/config/patrol_route.yaml";
  } catch (const std::exception &) {
    // Запасной абсолютный путь нужен для окружений, где пакет уже установлен,
    // но индекс пакетов недоступен или инициализирован не полностью.
    default_mission_file = "/home/ubuntu/ROS/install/patrolbot_mission_manager/share/"
      "patrolbot_mission_manager/config/patrol_route.yaml";
  }

  mission_file_ = declare_parameter<std::string>("mission_file", default_mission_file);
  status_topic_ = declare_parameter<std::string>("status_topic", "/patrol/status");
  nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
  status_publish_rate_hz_ = declare_parameter<double>("status_publish_rate_hz", 1.0);
  nav2_wait_timeout_sec_ = declare_parameter<double>("nav2_wait_timeout_sec", 30.0);
  verbose_logging_ = declare_parameter<bool>("verbose_logging", false);
  log_level_ = declare_parameter<std::string>("log_level", "info");

  if (status_publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("status_publish_rate_hz должно быть больше нуля");
  }

  if (nav2_wait_timeout_sec_ <= 0.0) {
    throw std::runtime_error("nav2_wait_timeout_sec должно быть больше нуля");
  }
}

bool MissionManagerNode::ReloadMissionConfiguration(std::string * error_text)
{
  try {
    // Маршрут загружается целиком и считается атомарным объектом конфигурации.
    // Если YAML невалиден, новый маршрут не принимается вообще.
    mission_config_ = patrolbot_utils::LoadMissionConfig(mission_file_);
    mission_config_loaded_ = true;
    return true;
  } catch (const std::exception & exception) {
    mission_config_loaded_ = false;
    if (error_text != nullptr) {
      *error_text = exception.what();
    }
    return false;
  }
}

bool MissionManagerNode::StartMissionInternal(const std::string & trigger, std::string * message)
{
  // Временное состояние предыдущей миссии должно быть полностью очищено до
  // начала нового запуска, иначе старые таймеры и старые action-результаты
  // смогут вмешаться в новый проход маршрута.
  pending_restart_ = false;
  CancelTimers();
  active_goal_handle_.reset();
  cancel_due_to_timeout_ = false;
  cancel_due_to_stop_ = false;

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "миссия", "запрошен запуск миссии, источник: " + trigger).c_str());

  state_machine_.SetValidatingRoute();
  PublishStatus();

  std::string error_text;
  // Маршрут перечитывается на каждый старт специально, чтобы оператор мог
  // менять YAML-миссию без пересборки и без перезапуска узла.
  if (!ReloadMissionConfiguration(&error_text)) {
    ApplyDecision(state_machine_.AbortMission("не удалось загрузить маршрут: " + error_text));
    if (message != nullptr) {
      *message = "ошибка загрузки маршрута: " + error_text;
    }
    return false;
  }

  // Mission manager не блокирует старт из-за того, что Nav2 ещё поднимается.
  // Если action-сервер пока недоступен, state machine переводит миссию в
  // состояние ожидания и таймер статуса доведёт её до запуска или до таймаута.
  const bool nav2_ready = action_client_->wait_for_action_server(std::chrono::seconds(0));
  ApplyDecision(state_machine_.ActivateMission(mission_config_, nav2_ready));

  if (message != nullptr) {
    *message = nav2_ready ?
      "миссия '" + mission_config_.name + "' запущена" :
      "миссия '" + mission_config_.name + "' ждёт action-сервер Nav2";
  }
  return true;
}

void MissionManagerNode::PublishStatus() const
{
  // Во внешний статус выгружается только "снимок" конечного автомата. Это
  // устраняет гонки между полями и делает интеграционные тесты детерминированными.
  patrolbot_interfaces::msg::PatrolStatus status;
  const auto & snapshot = state_machine_.snapshot();
  const auto * mission = state_machine_.mission();

  status.stamp = now();
  status.mission_name = mission != nullptr ? mission->name : mission_config_.name;
  status.state = ToString(snapshot.state);
  status.active = snapshot.active;
  status.current_waypoint_index = static_cast<std::uint32_t>(snapshot.current_waypoint_index);
  status.current_waypoint_name = CurrentWaypointName();
  status.completed_waypoints = static_cast<std::uint32_t>(snapshot.completed_waypoints);
  status.failed_waypoints = static_cast<std::uint32_t>(snapshot.failed_waypoints);
  status.loop_count = static_cast<std::uint32_t>(snapshot.loop_count);
  status.retry_count_for_current_waypoint =
    static_cast<std::uint32_t>(snapshot.retry_count_for_current_waypoint);
  status.last_error = snapshot.last_error;

  status_publisher_->publish(status);
}

void MissionManagerNode::StatusTimerCallback()
{
  PublishStatus();

  // Автозапуск откладывается до первого тика таймера, чтобы не инициировать
  // action-взаимодействие из конструктора узла.
  if (autostart_pending_ && state_machine_.snapshot().state == MissionState::kIdle) {
    autostart_pending_ = false;
    std::string message;
    StartMissionInternal("автозапуск", &message);
    return;
  }

  // В отдельном состоянии ожидания mission manager периодически проверяет,
  // поднялся ли Nav2, и завершает миссию ошибкой, если дедлайн ожидания вышел.
  if (state_machine_.snapshot().state == MissionState::kWaitingForNav2) {
    if (action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      ApplyDecision(state_machine_.HandleNav2Ready());
      return;
    }

    if (nav2_wait_deadline_active_ && now() > nav2_wait_deadline_) {
      nav2_wait_deadline_active_ = false;
      ApplyDecision(state_machine_.AbortMission("таймаут ожидания action-сервера Nav2"));
    }
  }
}

void MissionManagerNode::ApplyDecision(const TransitionDecision & decision)
{
  // Конечный автомат не знает о таймерах, паблишерах и action transport.
  // Этот метод является переводчиком из "решения автомата" в действие ROS 2.
  switch (decision.command) {
    case MissionCommand::kNone:
      break;

    case MissionCommand::kWaitForNav2:
      // Дедлайн ожидания живёт вне state machine, потому что это уже вопрос
      // работы с реальным временем, а не абстрактной доменной логики.
      nav2_wait_deadline_active_ = true;
      nav2_wait_deadline_ = now() + rclcpp::Duration::from_seconds(nav2_wait_timeout_sec_);
      RCLCPP_INFO(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage("миссия", decision.reason).c_str());
      break;

    case MissionCommand::kDispatchGoal:
      // Как только дошли до отправки цели, ожидание Nav2 больше неактуально.
      nav2_wait_deadline_active_ = false;
      DispatchCurrentGoal();
      break;

    case MissionCommand::kScheduleRetry:
      // Повтор делается не мгновенно, а через backoff-паузу. Это снижает
      // вероятность бесконтрольного цикла ошибок при временных сбоях.
      CancelTimers();
      if (const auto * mission = state_machine_.mission()) {
        const auto retry_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(mission->retry_backoff_sec));
        retry_timer_ = create_wall_timer(
          retry_delay,
          std::bind(&MissionManagerNode::RetryTimerCallback, this));
      }
      RCLCPP_WARN(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage("миссия", decision.reason).c_str());
      break;

    case MissionCommand::kCancelGoal:
      // Отмена активной цели централизована в отдельном методе, чтобы единообразно
      // проставлять причину отмены и корректно различать её в result callback.
      CancelActiveGoal(decision.reason);
      break;

    case MissionCommand::kCompleteMission:
      // После штатного завершения очищается всё временное состояние, чтобы
      // следующий запуск не унаследовал старые цели и таймеры.
      CancelTimers();
      active_goal_handle_.reset();
      RCLCPP_INFO(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage("миссия", decision.reason).c_str());
      break;

    case MissionCommand::kFailMission:
      // Для аварийного завершения выполняется тот же технический сброс, но
      // событие логируется как ошибка.
      CancelTimers();
      active_goal_handle_.reset();
      RCLCPP_ERROR(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage("ошибка", decision.reason).c_str());
      break;

    case MissionCommand::kStopped:
      // Сценарий "остановлено" используется как для ручной остановки, так и
      // для безопасного перезапуска миссии через stop -> start.
      CancelTimers();
      active_goal_handle_.reset();
      RCLCPP_INFO(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage("миссия", decision.reason).c_str());
      if (pending_restart_) {
        std::string restart_message;
        StartMissionInternal("перезапуск после остановки", &restart_message);
      }
      break;
  }

  PublishStatus();
}

void MissionManagerNode::DispatchCurrentGoal()
{
  // Узел всегда отправляет только одну текущую точку маршрута. Такой подход
  // упрощает контроль таймаутов, повторов и диагностики каждой waypoint.
  const auto * waypoint = state_machine_.CurrentWaypoint();
  const auto * mission = state_machine_.mission();
  if (waypoint == nullptr || mission == nullptr) {
    ApplyDecision(state_machine_.AbortMission("невозможно получить текущую точку маршрута"));
    return;
  }

  // Дополнительная проверка нужна на случай, если Nav2 был доступен в момент
  // активации миссии, но успел пропасть непосредственно перед отправкой цели.
  if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    ApplyDecision(state_machine_.AbortMission("action-сервер Nav2 недоступен"));
    return;
  }

  NavigateToPose::Goal goal;
  // Формирование PoseStamped вынесено в отдельный метод, чтобы вся логика
  // frame_id и преобразования yaw была сосредоточена в одном месте.
  goal.pose = BuildPoseStamped(*waypoint);

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "nav2",
      "отправляется цель '" + waypoint->name + "' [" +
      std::to_string(state_machine_.snapshot().current_waypoint_index) + "]").c_str());

  // Все callbacks привязываются при отправке конкретной цели, чтобы lifecycle
  // одной waypoint был описан рядом с местом её постановки в Nav2.
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MissionManagerNode::GoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &MissionManagerNode::FeedbackCallback,
    this,
    std::placeholders::_1,
    std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&MissionManagerNode::ResultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, send_goal_options);
}

void MissionManagerNode::GoalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle)
{
  // Отклонённая action-цель трактуется как полноценная ошибка прохождения точки
  // и передаётся в state machine для решения о повторе или остановке миссии.
  if (goal_handle == nullptr) {
    ApplyDecision(state_machine_.HandleGoalFailure("Nav2 отклонил цель"));
    return;
  }

  active_goal_handle_ = goal_handle;

  // Если оператор успел остановить миссию до подтверждения цели, новую цель
  // нельзя оставлять активной: она сразу отменяется.
  if (state_machine_.snapshot().state != MissionState::kNavigating) {
    cancel_due_to_stop_ = true;
    CancelActiveGoal("цель была принята после запроса остановки");
    return;
  }

  const auto * waypoint = state_machine_.CurrentWaypoint();
  if (waypoint == nullptr) {
    ApplyDecision(state_machine_.AbortMission("после принятия цели не найдена текущая точка"));
    return;
  }

  CancelTimers();
  // Таймер точки стартует только после того, как Nav2 действительно принял goal.
  // Это исключает ложные таймауты из-за задержек до приёма action-сообщения.
  const auto timeout_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(waypoint->timeout_sec));
  goal_timeout_timer_ = create_wall_timer(
    timeout_period,
    std::bind(&MissionManagerNode::GoalTimeoutCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "nav2",
      "цель принята, таймаут точки: " + std::to_string(waypoint->timeout_sec) + " сек").c_str());
}

void MissionManagerNode::FeedbackCallback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // Feedback не меняет состояние миссии и используется только для подробной
  // диагностики поведения Nav2 при сложных проездах и recovery.
  if (feedback == nullptr) {
    return;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "nav2",
      "feedback: distance_remaining=" + std::to_string(feedback->distance_remaining) +
      ", number_of_recoveries=" + std::to_string(feedback->number_of_recoveries)).c_str());
}

void MissionManagerNode::ResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  // Любой финальный результат action делает локальный таймер точки неактуальным.
  if (goal_timeout_timer_ != nullptr) {
    goal_timeout_timer_->cancel();
  }
  active_goal_handle_.reset();

  // Здесь transport-уровневые коды action переводятся в доменные события:
  // успех, отказ, остановка оператором или отмена по таймауту.
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      cancel_due_to_timeout_ = false;
      cancel_due_to_stop_ = false;
      ApplyDecision(state_machine_.HandleGoalSuccess());
      return;

    case rclcpp_action::ResultCode::ABORTED:
      cancel_due_to_timeout_ = false;
      cancel_due_to_stop_ = false;
      ApplyDecision(state_machine_.HandleGoalFailure("Nav2 вернул результат ABORTED"));
      return;

    case rclcpp_action::ResultCode::CANCELED:
      if (cancel_due_to_stop_) {
        cancel_due_to_stop_ = false;
        cancel_due_to_timeout_ = false;
        ApplyDecision(state_machine_.FinishStop("миссия остановлена оператором"));
        return;
      }

      if (cancel_due_to_timeout_) {
        cancel_due_to_timeout_ = false;
        ApplyDecision(state_machine_.HandleGoalTimeout());
        return;
      }

      ApplyDecision(state_machine_.HandleGoalFailure("цель отменена без явной причины"));
      return;

    default:
      cancel_due_to_timeout_ = false;
      cancel_due_to_stop_ = false;
      ApplyDecision(state_machine_.HandleGoalFailure("Nav2 вернул неизвестный результат"));
      return;
  }
}

void MissionManagerNode::GoalTimeoutCallback()
{
  if (goal_timeout_timer_ != nullptr) {
    goal_timeout_timer_->cancel();
  }

  // Если цель уже завершилась до срабатывания таймера, повторно ничего делать
  // не нужно: result callback уже обработал финальное состояние.
  if (active_goal_handle_ == nullptr) {
    return;
  }

  // Таймаут не завершает миссию напрямую, а инициирует штатную отмену action.
  // Причина отмены затем различается в ResultCallback по отдельному флагу.
  RCLCPP_ERROR(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "ошибка", "таймаут цели, отправляется запрос отмены в Nav2").c_str());
  cancel_due_to_timeout_ = true;
  cancel_due_to_stop_ = false;
  action_client_->async_cancel_goal(active_goal_handle_);
}

void MissionManagerNode::RetryTimerCallback()
{
  if (retry_timer_ != nullptr) {
    retry_timer_->cancel();
  }

  // После паузы между попытками решение снова принимается state machine, потому
  // что за это время оператор мог успеть остановить миссию.
  ApplyDecision(state_machine_.HandleRetryTimer());
}

void MissionManagerNode::CancelActiveGoal(const std::string & reason)
{
  CancelTimers();

  // Если активной цели уже нет, можно немедленно завершить сценарий остановки.
  if (active_goal_handle_ == nullptr) {
    ApplyDecision(state_machine_.FinishStop(reason));
    return;
  }

  // Флаг причины отмены нужен для корректной интерпретации status CANCELED
  // в callback результата action.
  cancel_due_to_stop_ = true;
  cancel_due_to_timeout_ = false;
  RCLCPP_WARN(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage("миссия", reason).c_str());
  action_client_->async_cancel_goal(active_goal_handle_);
}

geometry_msgs::msg::PoseStamped MissionManagerNode::BuildPoseStamped(
  const patrolbot_utils::WaypointConfig & waypoint) const
{
  // Waypoint всегда преобразуется в PoseStamped в координатной системе миссии.
  geometry_msgs::msg::PoseStamped pose;
  const auto * mission = state_machine_.mission();
  pose.header.stamp = now();
  pose.header.frame_id = mission != nullptr ? mission->frame_id : "map";
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;

  // В маршруте задаётся только yaw, поэтому roll и pitch принудительно нулевые.
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, waypoint.yaw_rad);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

std::string MissionManagerNode::CurrentWaypointName() const
{
  const auto * waypoint = state_machine_.CurrentWaypoint();
  return waypoint != nullptr ? waypoint->name : "";
}

void MissionManagerNode::CancelTimers()
{
  // Оба таймера независимы и могут существовать по отдельности.
  if (goal_timeout_timer_ != nullptr) {
    goal_timeout_timer_->cancel();
  }
  if (retry_timer_ != nullptr) {
    retry_timer_->cancel();
  }
}

void MissionManagerNode::StartServiceCallback(
  const std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Request> request,
  std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Response> response)
{
  // Повторный старт без явного разрешения на restart отвергается, чтобы не
  // допустить неоднозначное состояние для оператора и тестовой среды.
  if (state_machine_.snapshot().active && !request->restart_if_running) {
    response->accepted = false;
    response->message = "миссия уже выполняется";
    return;
  }

  // Перезапуск реализован как мягкая остановка текущей миссии с отложенным
  // повторным стартом после подтверждения остановки action-цели.
  if (state_machine_.snapshot().active && request->restart_if_running) {
    pending_restart_ = true;
    ApplyDecision(state_machine_.RequestStop("запрошен перезапуск миссии"));
    response->accepted = true;
    response->message = "текущая миссия останавливается и будет запущена заново";
    return;
  }

  response->accepted = StartMissionInternal("сервис /patrol/start", &response->message);
}

void MissionManagerNode::StopServiceCallback(
  const std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Request> request,
  std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Response> response)
{
  (void)request;

  // Запрос остановки уже неактивной миссии считается идемпотентной операцией.
  if (state_machine_.snapshot().state == MissionState::kIdle &&
    !state_machine_.snapshot().active)
  {
    response->accepted = true;
    response->message = "миссия уже остановлена";
    return;
  }

  pending_restart_ = false;
  ApplyDecision(state_machine_.RequestStop("получен запрос остановки миссии"));
  response->accepted = true;
  response->message = "запрос остановки принят";
}

}  // namespace patrolbot_mission_manager
