#ifndef PATROLBOT_BASE__MOTOR_DRIVER_HPP_
#define PATROLBOT_BASE__MOTOR_DRIVER_HPP_

#include <string>

namespace patrolbot_base
{

struct WheelCommand
{
  double left_rad_per_sec{0.0};
  double right_rad_per_sec{0.0};
};

struct WheelState
{
  double left_rad_per_sec{0.0};
  double right_rad_per_sec{0.0};
  bool connected{false};
  std::string status_message;
};

struct MotorDriverConfig
{
  std::string mode;
  std::string serial_port;
  int serial_baudrate{0};
  bool verbose_logging{false};
};

// Интерфейс отделяет механику и транспорт от логики ROS 2, чтобы код было проще тестировать.
class MotorDriver
{
public:
  virtual ~MotorDriver() = default;

  virtual bool Configure(const MotorDriverConfig & config) = 0;
  virtual bool Connect() = 0;
  virtual bool IsConnected() const = 0;
  virtual bool SendWheelCommand(const WheelCommand & command) = 0;
  virtual WheelState ReadState() = 0;
  virtual void Stop() = 0;
  virtual std::string Name() const = 0;
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__MOTOR_DRIVER_HPP_
