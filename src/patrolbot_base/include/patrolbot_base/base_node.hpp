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

// Параметры сведены в отдельную структуру, чтобы было проще
// передавать и тестировать конфигурацию базовой платформы.
struct BaseParameters
{
  // Радиус ведущего колеса в метрах.
  double wheel_radius{0.05};

  // Расстояние между центрами левого и правого колеса.
  double wheel_base{0.24};

  // Имя родительского фрейма одометрии.
  std::string odom_frame{"odom"};

  // Имя дочернего фрейма корпуса робота, относительно которого ведётся навигация.
  std::string base_frame{"base_footprint"};

  // Нужно ли публиковать TF odom -> base_frame из этого узла.
  bool publish_tf{true};

  // Через сколько миллисекунд без новых cmd_vel нужно остановить робота.
  int cmd_timeout_ms{500};

  // Ограничение линейной скорости для защиты от слишком агрессивных команд.
  double max_linear_velocity{0.5};

  // Ограничение угловой скорости.
  double max_angular_velocity{1.2};

  // Частота основного цикла управления и интеграции одометрии.
  double control_rate_hz{30.0};

  // Выбор транспорта к моторному контроллеру: mock или serial.
  std::string driver_mode{"mock"};

  // Последовательный порт контроллера моторов.
  std::string serial_port{"/dev/ttyUSB0"};

  // Скорость обмена по serial.
  int serial_baudrate{115200};

  // Дополнительные DEBUG-логи узла базы.
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
  // Считывает параметры ROS 2 в BaseParameters.
  void LoadParameters();

  // Проверяет, что конфигурация физически и логически допустима.
  void ValidateParameters() const;

  // Создаёт нужную реализацию MotorDriver в зависимости от режима.
  void CreateDriver();

  // Преобразует линейную и угловую скорость корпуса в угловые скорости колёс.
  WheelCommand ComputeWheelCommand(double linear_velocity, double angular_velocity) const;

  // Принимает команду движения, ограничивает её и запоминает как целевую.
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message);

  // Основной цикл управления: применяет watchdog, отправляет команды в драйвер,
  // обновляет одометрию и публикует все выходные сообщения.
  void ControlLoop();

  // Публикует nav_msgs/Odometry и, при необходимости, TF.
  void PublishOdometry(const rclcpp::Time & stamp, const OdometryState & state) const;

  // Публикует положение и скорости колёс в joint_states.
  void PublishJointState(const rclcpp::Time & stamp, const OdometryState & state) const;

  // Публикует компактную диагностику состояния драйвера и базовой платформы.
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

  // Последняя принятая целевая скорость корпуса.
  double target_linear_velocity_{0.0};
  double target_angular_velocity_{0.0};

  // Флаг не даёт повторно спамить одинаковым warning при каждой итерации watchdog.
  bool command_timed_out_{false};

  // Временные метки нужны для watchdog по cmd_vel и корректного расчёта dt интегратора.
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__BASE_NODE_HPP_
