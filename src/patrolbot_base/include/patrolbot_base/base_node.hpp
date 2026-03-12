#ifndef PATROLBOT_BASE__BASE_NODE_HPP_
#define PATROLBOT_BASE__BASE_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "patrolbot_base/motor_driver.hpp"
#include "patrolbot_base/odometry_integrator.hpp"

namespace patrolbot_base
{

struct BaseParameters
{
  double wheel_radius{0.05};
  double wheel_base{0.24};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  bool publish_tf{true};
  int cmd_timeout_ms{500};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.2};
  double control_rate_hz{30.0};
  std::string driver_mode{"mock"};
  std::string serial_port{"/dev/ttyUSB0"};
  int serial_baudrate{115200};
  bool verbose_logging{false};
  std::string log_level{"info"};
};

// Узел объединяет управление приводом, интеграцию одометрии и диагностические сообщения.
class BaseNode : public rclcpp::Node
{
public:
  BaseNode();
  ~BaseNode() override;

private:
  void LoadParameters();
  void ValidateParameters() const;
  void CreateDriver();
  WheelCommand ComputeWheelCommand(double linear_velocity, double angular_velocity) const;
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message);
  void ControlLoop();
  void PublishOdometry(const rclcpp::Time & stamp, const OdometryState & state) const;
  void PublishJointState(const rclcpp::Time & stamp, const OdometryState & state) const;
  void PublishDiagnostics(const rclcpp::Time & stamp, const WheelState & wheel_state) const;

  BaseParameters parameters_;
  std::unique_ptr<MotorDriver> motor_driver_;
  std::unique_ptr<OdometryIntegrator> odometry_integrator_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double target_linear_velocity_{0.0};
  double target_angular_velocity_{0.0};
  bool command_timed_out_{false};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__BASE_NODE_HPP_
