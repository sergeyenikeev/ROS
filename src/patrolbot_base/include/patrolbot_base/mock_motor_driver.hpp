#ifndef PATROLBOT_BASE__MOCK_MOTOR_DRIVER_HPP_
#define PATROLBOT_BASE__MOCK_MOTOR_DRIVER_HPP_

#include "patrolbot_base/motor_driver.hpp"

namespace patrolbot_base
{

// Mock-драйвер немедленно принимает команду как текущее состояние, что удобно для тестовой среды.
class MockMotorDriver : public MotorDriver
{
public:
  // Для mock-реализации конфигурация всегда принимается без реального I/O.
  bool Configure(const MotorDriverConfig & config) override;

  // Помечает драйвер как подключённый.
  bool Connect() override;
  bool IsConnected() const override;

  // Запоминает последнюю команду как текущее состояние тестовой базы.
  bool SendWheelCommand(const WheelCommand & command) override;

  // Возвращает состояние, синтезированное из последней принятой команды.
  WheelState ReadState() override;
  void Stop() override;
  std::string Name() const override;

private:
  MotorDriverConfig config_;
  WheelCommand last_command_;
  bool connected_{false};
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__MOCK_MOTOR_DRIVER_HPP_
