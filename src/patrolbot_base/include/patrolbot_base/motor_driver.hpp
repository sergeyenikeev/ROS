#ifndef PATROLBOT_BASE__MOTOR_DRIVER_HPP_
#define PATROLBOT_BASE__MOTOR_DRIVER_HPP_

#include <string>

namespace patrolbot_base
{

struct WheelCommand
{
  // Целевые угловые скорости левого и правого колеса в радианах в секунду.
  double left_rad_per_sec{0.0};
  double right_rad_per_sec{0.0};
};

struct WheelState
{
  // Измеренные или оценённые скорости колёс, которые драйвер возвращает базе.
  double left_rad_per_sec{0.0};
  double right_rad_per_sec{0.0};

  // Признак, что драйвер считает соединение с контроллером рабочим.
  bool connected{false};

  // Человекочитаемый статус для diagnostics и логов.
  std::string status_message;
};

struct MotorDriverConfig
{
  // Режим выбирает конкретную реализацию драйвера.
  std::string mode;

  // Параметры последовательного порта используются serial-реализацией.
  std::string serial_port;
  int serial_baudrate{0};

  // Подробное логирование может учитываться драйвером при отладке обмена.
  bool verbose_logging{false};
};

// Интерфейс отделяет механику и транспорт от логики ROS 2, чтобы код было проще тестировать.
class MotorDriver
{
public:
  virtual ~MotorDriver() = default;

  // Получает конфигурацию до попытки установить соединение.
  virtual bool Configure(const MotorDriverConfig & config) = 0;

  // Переводит драйвер в рабочее состояние.
  virtual bool Connect() = 0;

  // Позволяет верхнему уровню быстро проверить состояние транспорта.
  virtual bool IsConnected() const = 0;

  // Передаёт драйверу целевые скорости колёс.
  virtual bool SendWheelCommand(const WheelCommand & command) = 0;

  // Возвращает текущее наблюдаемое состояние колёс и соединения.
  virtual WheelState ReadState() = 0;

  // Останавливает движение и освобождает связанные ресурсы при необходимости.
  virtual void Stop() = 0;

  // Короткое имя реализации удобно для логов и диагностики.
  virtual std::string Name() const = 0;
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__MOTOR_DRIVER_HPP_
