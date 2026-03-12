#include "patrolbot_base/base_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "patrolbot_base/mock_motor_driver.hpp"
#include "patrolbot_base/serial_motor_driver.hpp"
#include "patrolbot_utils/diagnostic_helper.hpp"
#include "patrolbot_utils/logging_utils.hpp"
#include "patrolbot_utils/parameter_validation.hpp"

namespace patrolbot_base
{

namespace
{

constexpr char kLeftWheelJointName[] = "left_wheel_joint";
constexpr char kRightWheelJointName[] = "right_wheel_joint";

}  // namespace

// Узел базы отвечает за нижний уровень мобильной платформы: принимает cmd_vel,
// переводит их в команды колёсам, читает состояние драйвера, считает одометрию
// и публикует наружу odom, joint_states и диагностику.
BaseNode::BaseNode()
: Node("patrolbot_base")
{
  LoadParameters();
  ValidateParameters();
  CreateDriver();

  // Интегратор хранится отдельно от ROS 2 инфраструктуры, чтобы его можно было
  // полноценно тестировать как чистый математический компонент.
  odometry_integrator_ = std::make_unique<OdometryIntegrator>(
    parameters_.wheel_radius, parameters_.wheel_base);

  // Подписка на /cmd_vel и периодический control loop разделены: это позволяет
  // принимать команды асинхронно, а применять их с фиксированной частотой.
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    20,
    std::bind(&BaseNode::CmdVelCallback, this, std::placeholders::_1));

  if (parameters_.publish_tf) {
    // Публикация TF делается опциональной, чтобы узел можно было встроить в
    // стенды, где transform odom -> base уже выдаёт другой компонент.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  last_cmd_time_ = now();
  last_control_time_ = now();

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / parameters_.control_rate_hz));
  control_timer_ = create_wall_timer(period, std::bind(&BaseNode::ControlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "узел",
      "patrolbot_base запущен, режим драйвера: " + motor_driver_->Name()).c_str());
}

BaseNode::~BaseNode()
{
  // Даже при исключениях и штатном завершении драйвер должен получить нулевую
  // команду, поэтому остановка вынесена в деструктор.
  if (motor_driver_ != nullptr) {
    motor_driver_->Stop();
  }

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage("узел", "patrolbot_base остановлен").c_str());
}

void BaseNode::LoadParameters()
{
  // Все параметры читаются централизованно, чтобы диагностика конфигурации и
  // unit-тесты работали с одной и той же структурой BaseParameters.
  parameters_.wheel_radius = declare_parameter<double>("wheel_radius", parameters_.wheel_radius);
  parameters_.wheel_base = declare_parameter<double>("wheel_base", parameters_.wheel_base);
  parameters_.odom_frame = declare_parameter<std::string>("odom_frame", parameters_.odom_frame);
  parameters_.base_frame = declare_parameter<std::string>("base_frame", parameters_.base_frame);
  parameters_.publish_tf = declare_parameter<bool>("publish_tf", parameters_.publish_tf);
  parameters_.cmd_timeout_ms =
    declare_parameter<int>("cmd_timeout_ms", parameters_.cmd_timeout_ms);
  parameters_.max_linear_velocity = declare_parameter<double>(
    "max_linear_velocity", parameters_.max_linear_velocity);
  parameters_.max_angular_velocity = declare_parameter<double>(
    "max_angular_velocity", parameters_.max_angular_velocity);
  parameters_.control_rate_hz = declare_parameter<double>(
    "control_rate_hz", parameters_.control_rate_hz);
  parameters_.driver_mode = declare_parameter<std::string>("driver_mode", parameters_.driver_mode);
  parameters_.serial_port = declare_parameter<std::string>("serial_port", parameters_.serial_port);
  parameters_.serial_baudrate =
    declare_parameter<int>("serial_baudrate", parameters_.serial_baudrate);
  parameters_.verbose_logging =
    declare_parameter<bool>("verbose_logging", parameters_.verbose_logging);
  parameters_.log_level = declare_parameter<std::string>("log_level", parameters_.log_level);

  RCLCPP_INFO(
    get_logger(),
    "%s",
    patrolbot_utils::MakeLogMessage(
      "параметры",
      "загружены параметры базы: wheel_radius=" + std::to_string(parameters_.wheel_radius) +
      ", wheel_base=" + std::to_string(parameters_.wheel_base) +
      ", driver_mode=" + parameters_.driver_mode).c_str());
}

void BaseNode::ValidateParameters() const
{
  // Валидация выполняется до создания драйвера и таймеров, чтобы узел не жил
  // даже короткое время с физически невозможной конфигурацией.
  patrolbot_utils::ValidationIssues issues;

  patrolbot_utils::RequirePositive("wheel_radius", parameters_.wheel_radius, &issues);
  patrolbot_utils::RequirePositive("wheel_base", parameters_.wheel_base, &issues);
  patrolbot_utils::RequirePositive("cmd_timeout_ms", parameters_.cmd_timeout_ms, &issues);
  patrolbot_utils::RequirePositive("control_rate_hz", parameters_.control_rate_hz, &issues);
  patrolbot_utils::RequirePositive(
    "max_linear_velocity", parameters_.max_linear_velocity, &issues);
  patrolbot_utils::RequirePositive(
    "max_angular_velocity", parameters_.max_angular_velocity, &issues);
  patrolbot_utils::RequireNotEmpty("odom_frame", parameters_.odom_frame, &issues);
  patrolbot_utils::RequireNotEmpty("base_frame", parameters_.base_frame, &issues);
  patrolbot_utils::RequireNotEmpty("driver_mode", parameters_.driver_mode, &issues);

  if (parameters_.driver_mode != "mock" && parameters_.driver_mode != "serial") {
    issues.push_back({"driver_mode", "допустимы только значения mock или serial"});
  }

  if (parameters_.driver_mode == "serial") {
    patrolbot_utils::RequireNotEmpty("serial_port", parameters_.serial_port, &issues);
    patrolbot_utils::RequirePositive("serial_baudrate", parameters_.serial_baudrate, &issues);
  }

  if (patrolbot_utils::HasIssues(issues)) {
    const auto text = patrolbot_utils::FormatIssues(issues);
    RCLCPP_FATAL(
      get_logger(),
      "%s",
      patrolbot_utils::MakeLogMessage(
        "ошибка", "некорректная конфигурация patrolbot_base: " + text).c_str());
    throw std::runtime_error(text);
  }
}

void BaseNode::CreateDriver()
{
  // Конфигурация драйвера собирается из параметров узла, но сам BaseNode не
  // знает деталей конкретной транспортной реализации.
  MotorDriverConfig config;
  config.mode = parameters_.driver_mode;
  config.serial_port = parameters_.serial_port;
  config.serial_baudrate = parameters_.serial_baudrate;
  config.verbose_logging = parameters_.verbose_logging;

  // В v1 используются две реализации: полностью тестовый mock и безопасная
  // serial-заготовка под реальное железо.
  if (parameters_.driver_mode == "mock") {
    motor_driver_ = std::make_unique<MockMotorDriver>();
  } else {
    motor_driver_ = std::make_unique<SerialMotorDriver>();
  }

  // Configure отвечает за валидацию внутренних параметров драйвера, а Connect —
  // за перевод его в рабочее состояние.
  if (!motor_driver_->Configure(config)) {
    throw std::runtime_error("не удалось настроить драйвер моторов");
  }

  if (!motor_driver_->Connect()) {
    throw std::runtime_error("не удалось подключить драйвер моторов");
  }
}

WheelCommand BaseNode::ComputeWheelCommand(
  double linear_velocity,
  double angular_velocity) const
{
  // Для дифференциального привода линейная скорость корпуса раскладывается
  // в линейные скорости левого и правого колеса относительно центра базы.
  const double left_linear =
    linear_velocity - angular_velocity * parameters_.wheel_base * 0.5;
  const double right_linear =
    linear_velocity + angular_velocity * parameters_.wheel_base * 0.5;

  // Драйвер ожидает угловые скорости колёс, поэтому линейные скорости
  // переводятся через радиус колеса.
  WheelCommand command;
  command.left_rad_per_sec = left_linear / parameters_.wheel_radius;
  command.right_rad_per_sec = right_linear / parameters_.wheel_radius;
  return command;
}

void BaseNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  // Команда ограничивается по безопасным пределам до записи в целевые поля.
  const double clamped_linear = std::clamp(
    message->linear.x,
    -parameters_.max_linear_velocity,
    parameters_.max_linear_velocity);
  const double clamped_angular = std::clamp(
    message->angular.z,
    -parameters_.max_angular_velocity,
    parameters_.max_angular_velocity);

  // Предупреждение выводится только при verbose-режиме, чтобы не засорять логи
  // на каждой команде, выходящей за пределы допустимых скоростей.
  if ((clamped_linear != message->linear.x || clamped_angular != message->angular.z) &&
    parameters_.verbose_logging)
  {
    RCLCPP_WARN(
      get_logger(),
      "%s",
      patrolbot_utils::MakeLogMessage(
        "база", "команда движения была ограничена параметрами безопасности").c_str());
  }

  target_linear_velocity_ = clamped_linear;
  target_angular_velocity_ = clamped_angular;
  command_timed_out_ = false;
  last_cmd_time_ = now();
}

void BaseNode::ControlLoop()
{
  // dt вычисляется по реальному времени, а не считается идеальным, чтобы
  // интегратор оставался устойчивым даже при неравномерном вызове таймера.
  const rclcpp::Time current_time = now();
  double dt_sec = (current_time - last_control_time_).seconds();
  if (dt_sec <= 0.0) {
    dt_sec = 1.0 / parameters_.control_rate_hz;
  }
  last_control_time_ = current_time;

  // По умолчанию применяется последняя целевая скорость, пришедшая из /cmd_vel.
  double linear_velocity = target_linear_velocity_;
  double angular_velocity = target_angular_velocity_;
  const auto since_last_cmd = (current_time - last_cmd_time_).nanoseconds() / 1000000;

  // Watchdog защищает робот от ситуации, когда верхний уровень перестал
  // публиковать команды, а база продолжает ехать по старому значению.
  if (since_last_cmd > parameters_.cmd_timeout_ms) {
    linear_velocity = 0.0;
    angular_velocity = 0.0;

    if (!command_timed_out_) {
      RCLCPP_WARN(
        get_logger(),
        "%s",
        patrolbot_utils::MakeLogMessage(
          "база", "сработал watchdog по /cmd_vel, команда сброшена в ноль").c_str());
      command_timed_out_ = true;
    }
  }

  // После применения watchdog вычисляется команда для колёс и отправляется
  // в драйвер на каждом такте control loop.
  const WheelCommand command = ComputeWheelCommand(linear_velocity, angular_velocity);
  if (!motor_driver_->SendWheelCommand(command)) {
    RCLCPP_ERROR(
      get_logger(),
      "%s",
      patrolbot_utils::MakeLogMessage(
        "ошибка", "драйвер моторов не принял команду движения").c_str());
  }

  // База читает текущее состояние колёс обратно из драйвера и использует его
  // для интеграции одометрии, а не полагается только на отправленную команду.
  const WheelState wheel_state = motor_driver_->ReadState();
  const OdometryState odometry_state =
    odometry_integrator_->Step(
    wheel_state.left_rad_per_sec, wheel_state.right_rad_per_sec, dt_sec);

  // Публикации собраны в конце цикла, чтобы все сообщения относились к одному
  // и тому же измерению времени и одному состоянию платформы.
  PublishOdometry(current_time, odometry_state);
  PublishJointState(current_time, odometry_state);
  PublishDiagnostics(current_time, wheel_state);

  if (parameters_.verbose_logging) {
    RCLCPP_DEBUG(
      get_logger(),
      "%s",
      patrolbot_utils::MakeLogMessage(
        "база",
        "опубликованы odom и diagnostics, x=" + std::to_string(odometry_state.x) +
        ", y=" + std::to_string(odometry_state.y) +
        ", yaw=" + std::to_string(odometry_state.yaw)).c_str());
  }
}

void BaseNode::PublishOdometry(const rclcpp::Time & stamp, const OdometryState & state) const
{
  // Одометрия публикуется в том же frame_id, который использует навигационный
  // стек. Это делает базовый узел независимым от конкретного названия фреймов.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = parameters_.odom_frame;
  odom.child_frame_id = parameters_.base_frame;
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;

  // Ориентация корпуса хранится как yaw, но в ROS сообщении должна быть задана
  // полным кватернионом.
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, state.yaw);
  odom.pose.pose.orientation = tf2::toMsg(quaternion);
  odom.twist.twist.linear.x = state.linear_velocity;
  odom.twist.twist.angular.z = state.angular_velocity;
  odom_publisher_->publish(odom);

  // TF публикуется из того же источника данных, что и /odom, чтобы не было
  // рассогласования между сообщением и transform.
  if (parameters_.publish_tf && tf_broadcaster_ != nullptr) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parameters_.odom_frame;
    transform.child_frame_id = parameters_.base_frame;
    transform.transform.translation.x = state.x;
    transform.transform.translation.y = state.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

void BaseNode::PublishJointState(const rclcpp::Time & stamp, const OdometryState & state) const
{
  // Скорости joint_states вычисляются из текущей скорости корпуса. Для mock и
  // заготовочного serial-драйвера этого достаточно, чтобы дать связный поток
  // данных в robot_state_publisher и средства визуализации.
  const WheelCommand wheel_velocity_command =
    ComputeWheelCommand(state.linear_velocity, state.angular_velocity);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name = {kLeftWheelJointName, kRightWheelJointName};
  joint_state.position = {state.left_wheel_position, state.right_wheel_position};
  joint_state.velocity = {
    wheel_velocity_command.left_rad_per_sec,
    wheel_velocity_command.right_rad_per_sec};
  joint_state_publisher_->publish(joint_state);
}

void BaseNode::PublishDiagnostics(
  const rclcpp::Time & stamp,
  const WheelState & wheel_state) const
{
  // Диагностика упрощена до одного агрегированного статуса базы, но при этом
  // содержит ключевые значения для быстрой отладки на стенде и на железе.
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  const std::uint8_t level = wheel_state.connected ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;

  array.status.push_back(patrolbot_utils::MakeDiagnosticStatus(
    "patrolbot_base",
    level,
    wheel_state.status_message,
    {
      {"driver_mode", parameters_.driver_mode},
      {"left_rad_per_sec", std::to_string(wheel_state.left_rad_per_sec)},
      {"right_rad_per_sec", std::to_string(wheel_state.right_rad_per_sec)},
      {"publish_tf", parameters_.publish_tf ? "true" : "false"},
    }));

  diagnostics_publisher_->publish(array);
}

}  // namespace patrolbot_base
