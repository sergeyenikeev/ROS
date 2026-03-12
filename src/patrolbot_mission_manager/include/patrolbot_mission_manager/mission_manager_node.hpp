#ifndef PATROLBOT_MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_
#define PATROLBOT_MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "patrolbot_interfaces/msg/patrol_status.hpp"
#include "patrolbot_interfaces/srv/start_patrol.hpp"
#include "patrolbot_interfaces/srv/stop_patrol.hpp"
#include "patrolbot_utils/route_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "patrolbot_mission_manager/mission_state_machine.hpp"

namespace patrolbot_mission_manager
{

// Узел mission manager координирует прикладной сценарий патрулирования поверх Nav2.
// Он не занимается построением карты или локализацией, а только:
// - загружает и проверяет YAML-миссию;
// - следит за доступностью action-сервера навигации;
// - последовательно отправляет цели в Nav2;
// - обрабатывает таймауты, повторы, остановку и перезапуск;
// - публикует понятный наружу статус миссии.
class MissionManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionManagerNode();

private:
  // Загружает параметры ROS 2 и сразу валидирует критичные ограничения, чтобы
  // узел не стартовал в полу-рабочем состоянии с некорректной конфигурацией.
  void LoadParameters();

  // Перечитывает YAML-миссию с диска. Это вызывается перед каждым стартом миссии,
  // чтобы оператор мог менять маршрут без пересборки пакета.
  bool ReloadMissionConfiguration(std::string * error_text);

  // Единая точка входа для старта миссии из сервиса, автозапуска или перезапуска.
  // Здесь сбрасываются внутренние флаги, перечитывается конфигурация и выбирается,
  // можно ли сразу отправлять первую цель или нужно ждать Nav2.
  bool StartMissionInternal(const std::string & trigger, std::string * message);

  // Формирует и публикует внешний статус, который могут читать операторские инструменты,
  // тесты или надстройки верхнего уровня.
  void PublishStatus() const;

  // Периодический heartbeat узла:
  // - публикует статус;
  // - выполняет отложенный автозапуск;
  // - проверяет, появился ли action Nav2;
  // - завершает миссию, если ожидание Nav2 превысило допустимый лимит.
  void StatusTimerCallback();

  // Преобразует абстрактное решение state machine в реальные действия узла:
  // отправку цели, запуск таймера повтора, остановку или завершение миссии.
  void ApplyDecision(const TransitionDecision & decision);

  // Отправляет текущую точку маршрута в Nav2 через action NavigateToPose.
  void DispatchCurrentGoal();

  // Получает результат приёма цели action-сервером.
  void GoalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle);

  // Обрабатывает feedback от Nav2. Здесь хранится только логирование и диагностика,
  // чтобы state machine не зависела от деталей транспортного уровня action.
  void FeedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  // Обрабатывает финальный результат action и переводит его в событие state machine.
  void ResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);

  // Срабатывает, если точка не была пройдена за отведённое время.
  void GoalTimeoutCallback();

  // Срабатывает после задержки между попытками.
  void RetryTimerCallback();

  // Отправляет отмену активной цели в Nav2 и помечает причину отмены.
  void CancelActiveGoal(const std::string & reason);

  // Преобразует waypoint в PoseStamped в нужном frame_id миссии.
  geometry_msgs::msg::PoseStamped BuildPoseStamped(
    const patrolbot_utils::WaypointConfig & waypoint) const;

  // Возвращает имя текущей точки для внешнего статуса и логов.
  std::string CurrentWaypointName() const;

  // Останавливает таймер таймаута и таймер повтора, если они были активны.
  void CancelTimers();

  // Сервисный обработчик старта миссии.
  void StartServiceCallback(
    const std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Request> request,
    std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Response> response);

  // Сервисный обработчик остановки миссии.
  void StopServiceCallback(
    const std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Request> request,
    std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Response> response);

  // Путь до YAML-файла маршрута. Оставлен параметром, чтобы подменять миссии без правки кода.
  std::string mission_file_;

  // Топик публикации статуса патруля.
  std::string status_topic_;

  // Имя action-сервера Nav2. Вынесено в параметр для упрощения интеграции с нестандартными стендами.
  std::string nav2_action_name_;

  // Частота публикации статуса наружу.
  double status_publish_rate_hz_{1.0};

  // Максимальное время, которое mission manager готов ждать появления Nav2 после старта миссии.
  double nav2_wait_timeout_sec_{30.0};

  // Флаг включает дополнительные DEBUG-логи в чувствительных участках.
  bool verbose_logging_{false};
  std::string log_level_{"info"};

  // Последняя успешно загруженная конфигурация миссии.
  patrolbot_utils::MissionConfig mission_config_;

  // Флаг отделяет "миссия ещё ни разу не загружалась" от "миссия есть, но сейчас неактивна".
  bool mission_config_loaded_{false};

  // Если YAML требует автозапуска, узел не стартует миссию прямо в конструкторе,
  // а откладывает старт до первого тика таймера статуса.
  bool autostart_pending_{false};

  // Используется при сценарии "остановить текущую миссию и тут же запустить заново".
  bool pending_restart_{false};

  // Пока mission manager ждёт Nav2, здесь хранится активность и дедлайн ожидания.
  bool nav2_wait_deadline_active_{false};
  rclcpp::Time nav2_wait_deadline_;

  // Эти флаги нужны, чтобы различать отмену по таймауту и отмену по команде оператора,
  // хотя обе внешне приходят как action result со статусом CANCELED.
  bool cancel_due_to_timeout_{false};
  bool cancel_due_to_stop_{false};

  // Чистая state machine хранит правила переходов и не знает про ROS transport.
  MissionStateMachine state_machine_;

  // Паблишеры, сервисы, action-клиент и таймеры составляют "обвязку" над state machine.
  rclcpp::Publisher<patrolbot_interfaces::msg::PatrolStatus>::SharedPtr status_publisher_;
  rclcpp::Service<patrolbot_interfaces::srv::StartPatrol>::SharedPtr start_service_;
  rclcpp::Service<patrolbot_interfaces::srv::StopPatrol>::SharedPtr stop_service_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace patrolbot_mission_manager

#endif  // PATROLBOT_MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_
