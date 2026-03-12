#ifndef PATROLBOT_BASE__SERIAL_MOTOR_DRIVER_HPP_
#define PATROLBOT_BASE__SERIAL_MOTOR_DRIVER_HPP_

#include "patrolbot_base/motor_driver.hpp"

namespace patrolbot_base
{

// Заготовка под реальный драйвер по последовательному порту.
// В текущей среде узел ведёт себя безопасно, но не общается с внешним контроллером.
class SerialMotorDriver : public MotorDriver
{
public:
  // Проверяет базовую корректность параметров последовательного соединения.
  bool Configure(const MotorDriverConfig & config) override;

  // Переводит драйвер в логическое состояние "подключён".
  bool Connect() override;
  bool IsConnected() const override;

  // Отправляет или, на текущем этапе, сохраняет целевую команду колёсам.
  bool SendWheelCommand(const WheelCommand & command) override;

  // Возвращает последнее известное состояние колёс и диагностический статус.
  WheelState ReadState() override;
  void Stop() override;
  std::string Name() const override;

private:
  MotorDriverConfig config_;
  WheelCommand last_command_;
  bool connected_{false};
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__SERIAL_MOTOR_DRIVER_HPP_
