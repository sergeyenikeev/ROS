#ifndef PATROLBOT_BASE__MOCK_MOTOR_DRIVER_HPP_
#define PATROLBOT_BASE__MOCK_MOTOR_DRIVER_HPP_

#include "patrolbot_base/motor_driver.hpp"

namespace patrolbot_base
{

// Mock-драйвер немедленно принимает команду как текущее состояние, что удобно для тестовой среды.
class MockMotorDriver : public MotorDriver
{
public:
  bool Configure(const MotorDriverConfig & config) override;
  bool Connect() override;
  bool IsConnected() const override;
  bool SendWheelCommand(const WheelCommand & command) override;
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
