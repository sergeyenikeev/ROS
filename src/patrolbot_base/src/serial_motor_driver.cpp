#include "patrolbot_base/serial_motor_driver.hpp"

namespace patrolbot_base
{

bool SerialMotorDriver::Configure(const MotorDriverConfig & config)
{
  config_ = config;
  return !config_.serial_port.empty() && config_.serial_baudrate > 0;
}

bool SerialMotorDriver::Connect()
{
  // Здесь намеренно нет реального I/O: код остаётся безопасной заготовкой до привязки к железу.
  connected_ = !config_.serial_port.empty() && config_.serial_baudrate > 0;
  return connected_;
}

bool SerialMotorDriver::IsConnected() const
{
  return connected_;
}

bool SerialMotorDriver::SendWheelCommand(const WheelCommand & command)
{
  if (!connected_) {
    return false;
  }

  last_command_ = command;
  return true;
}

WheelState SerialMotorDriver::ReadState()
{
  WheelState state;
  state.left_rad_per_sec = last_command_.left_rad_per_sec;
  state.right_rad_per_sec = last_command_.right_rad_per_sec;
  state.connected = connected_;
  state.status_message =
    "serial-драйвер работает в режиме заготовки и требует адаптации под контроллер моторов";
  return state;
}

void SerialMotorDriver::Stop()
{
  last_command_ = WheelCommand{};
}

std::string SerialMotorDriver::Name() const
{
  return "serial";
}

}  // namespace patrolbot_base
