#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace patrolbot_system_tests
{

// Тестовый action-сервер имитирует минимально необходимое поведение Nav2.
// Он нужен для интеграционных тестов mission manager без запуска полного
// навигационного стека и без реального planner/controller.
class MockNav2ActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  MockNav2ActionServer()
  : Node("mock_nav2_action_server")
  {
    // result_sequence задаёт сценарий результатов по очереди:
    // succeed, abort, timeout и их комбинации через запятую.
    const auto result_sequence = declare_parameter<std::string>("result_sequence", "succeed");
    result_delay_ms_ = declare_parameter<int>("result_delay_ms", 300);
    feedback_period_ms_ = declare_parameter<int>("feedback_period_ms", 100);
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
    verbose_logging_ = declare_parameter<bool>("verbose_logging", false);

    // Последовательность разбирается один раз при старте узла, чтобы дальше
    // Execute работал только с готовым массивом outcomes.
    std::stringstream sequence_stream(result_sequence);
    std::string token;
    while (std::getline(sequence_stream, token, ',')) {
      token.erase(
        std::remove_if(token.begin(), token.end(), [](unsigned char symbol) {return symbol == ' ';}),
        token.end());
      if (!token.empty()) {
        result_sequence_.push_back(token);
      }
    }
    if (result_sequence_.empty()) {
      result_sequence_.push_back("succeed");
    }

    // Action-сервер публикуется под тем же именем, которое ожидает mission manager.
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      nav2_action_name_,
      std::bind(&MockNav2ActionServer::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MockNav2ActionServer::HandleCancel, this, std::placeholders::_1),
      std::bind(&MockNav2ActionServer::HandleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "mock_nav2_action_server запущен, result_sequence=%s",
      result_sequence.c_str());
  }

private:
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    // Для тестов goal всегда принимается: проверяется не фильтрация целей,
    // а реакция mission manager на результат выполнения.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandleNavigateToPose>)
  {
    // Отмена всегда разрешается, чтобы тесты таймаута и stop-поведения были
    // детерминированными и не зависели от дополнительной логики action-сервера.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    // Каждая цель исполняется в отдельном потоке, как это обычно делает action
    // сервер с длительным выполнением.
    std::thread{std::bind(&MockNav2ActionServer::Execute, this, goal_handle)}.detach();
  }

  void Execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    // Сценарий результата выбирается циклически по номеру цели. Это позволяет
    // одной строкой параметра описывать длинные тестовые последовательности.
    const std::size_t index = goal_counter_.fetch_add(1U);
    const std::string outcome = result_sequence_[index % result_sequence_.size()];
    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    const auto started_at = now();
    while (rclcpp::ok()) {
      // Если mission manager запросил отмену, goal немедленно завершается
      // статусом canceled, чтобы тест мог проверить реакцию автомата.
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->canceled(result);
        return;
      }

      // При timeout-сценарии feedback продолжает идти, но финальный result не
      // публикуется. Это позволяет проверить собственный watchdog mission manager.
      feedback->distance_remaining = outcome == "timeout" ? 1.0F : 0.5F;
      feedback->number_of_recoveries = 0;
      goal_handle->publish_feedback(feedback);

      const auto elapsed_ms = (now() - started_at).nanoseconds() / 1000000;
      if (outcome != "timeout" && elapsed_ms >= result_delay_ms_) {
        auto result = std::make_shared<NavigateToPose::Result>();
        // abort нужен для сценариев повторной попытки, succeed - для штатного
        // прохождения маршрута.
        if (outcome == "abort") {
          goal_handle->abort(result);
        } else {
          goal_handle->succeed(result);
        }
        return;
      }

      if (verbose_logging_) {
        RCLCPP_DEBUG(
          get_logger(),
          "mock_nav2_action_server outcome=%s elapsed_ms=%lld",
          outcome.c_str(),
          static_cast<long long>(elapsed_ms));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(feedback_period_ms_));
    }
  }

  std::vector<std::string> result_sequence_;
  int result_delay_ms_{300};
  int feedback_period_ms_{100};
  std::string nav2_action_name_;
  bool verbose_logging_{false};
  std::atomic<std::size_t> goal_counter_{0U};
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
};

}  // namespace patrolbot_system_tests

int main(int argc, char ** argv)
{
  // Отдельный исполняемый файл нужен только для тестовой среды launch_testing.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<patrolbot_system_tests::MockNav2ActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
