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

class MissionManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionManagerNode();

private:
  void LoadParameters();
  bool ReloadMissionConfiguration(std::string * error_text);
  bool StartMissionInternal(const std::string & trigger, std::string * message);
  void PublishStatus() const;
  void StatusTimerCallback();
  void ApplyDecision(const TransitionDecision & decision);
  void DispatchCurrentGoal();
  void GoalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle);
  void FeedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void ResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
  void GoalTimeoutCallback();
  void RetryTimerCallback();
  void CancelActiveGoal(const std::string & reason);
  geometry_msgs::msg::PoseStamped BuildPoseStamped(
    const patrolbot_utils::WaypointConfig & waypoint) const;
  std::string CurrentWaypointName() const;
  void CancelTimers();

  void StartServiceCallback(
    const std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Request> request,
    std::shared_ptr<patrolbot_interfaces::srv::StartPatrol::Response> response);
  void StopServiceCallback(
    const std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Request> request,
    std::shared_ptr<patrolbot_interfaces::srv::StopPatrol::Response> response);

  std::string mission_file_;
  std::string status_topic_;
  std::string nav2_action_name_;
  double status_publish_rate_hz_{1.0};
  double nav2_wait_timeout_sec_{30.0};
  bool verbose_logging_{false};
  std::string log_level_{"info"};

  patrolbot_utils::MissionConfig mission_config_;
  bool mission_config_loaded_{false};
  bool autostart_pending_{false};
  bool pending_restart_{false};
  bool nav2_wait_deadline_active_{false};
  rclcpp::Time nav2_wait_deadline_;
  bool cancel_due_to_timeout_{false};
  bool cancel_due_to_stop_{false};

  MissionStateMachine state_machine_;

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
