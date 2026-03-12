#include "patrolbot_base/mock_motor_driver.hpp"

namespace patrolbot_base
{

bool MockMotorDriver::Configure(const MotorDriverConfig & config)
{
  config_ = config;
  return true;
}

bool MockMotorDriver::Connect()
{
  connected_ = true;
  return true;
}

bool MockMotorDriver::IsConnected() const
{
  return connected_;
}

bool MockMotorDriver::SendWheelCommand(const WheelCommand & command)
{
  if (!connected_) {
    return false;
  }

  last_command_ = command;
  return true;
}

WheelState MockMotorDriver::ReadState()
{
  WheelState state;
  state.left_rad_per_sec = last_command_.left_rad_per_sec;
  state.right_rad_per_sec = last_command_.right_rad_per_sec;
  state.connected = connected_;
  state.status_message = connected_ ? "mock-драйвер активен" : "mock-драйвер не подключён";
  return state;
}

void MockMotorDriver::Stop()
{
  last_command_ = WheelCommand{};
}

std::string MockMotorDriver::Name() const
{
  return "mock";
}

}  // namespace patrolbot_base
