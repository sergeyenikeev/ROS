#include "patrolbot_base/mock_motor_driver.hpp"

namespace patrolbot_base
{

bool MockMotorDriver::Configure(const MotorDriverConfig & config)
{
  // Mock-драйвер принимает конфигурацию без реальной проверки транспорта:
  // он нужен прежде всего для тестов и для режима bringup без железа.
  config_ = config;
  return true;
}

bool MockMotorDriver::Connect()
{
  // В тестовой реализации "подключение" всегда мгновенное и успешное.
  connected_ = true;
  return true;
}

bool MockMotorDriver::IsConnected() const
{
  return connected_;
}

bool MockMotorDriver::SendWheelCommand(const WheelCommand & command)
{
  // Если mock не переведён в состояние connected, он ведёт себя так же, как
  // реальный драйвер, и отвергает команду.
  if (!connected_) {
    return false;
  }

  last_command_ = command;
  return true;
}

WheelState MockMotorDriver::ReadState()
{
  // Mock просто отражает последнюю принятую команду как текущее состояние
  // колёс. Этого достаточно для интеграционных тестов логики базы.
  WheelState state;
  state.left_rad_per_sec = last_command_.left_rad_per_sec;
  state.right_rad_per_sec = last_command_.right_rad_per_sec;
  state.connected = connected_;
  state.status_message = connected_ ? "mock-драйвер активен" : "mock-драйвер не подключён";
  return state;
}

void MockMotorDriver::Stop()
{
  // Остановка сбрасывает последнее движение в ноль, но не рвёт само подключение.
  last_command_ = WheelCommand{};
}

std::string MockMotorDriver::Name() const
{
  return "mock";
}

}  // namespace patrolbot_base
