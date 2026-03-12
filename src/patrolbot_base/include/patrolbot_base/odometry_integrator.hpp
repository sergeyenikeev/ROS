#ifndef PATROLBOT_BASE__ODOMETRY_INTEGRATOR_HPP_
#define PATROLBOT_BASE__ODOMETRY_INTEGRATOR_HPP_

namespace patrolbot_base
{

struct OdometryState
{
  // Текущая оценка положения робота в плоскости odom.
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  // Линейная и угловая скорость корпуса, вычисленные из скоростей колёс.
  double linear_velocity{0.0};
  double angular_velocity{0.0};

  // Накопленные углы вращения колёс нужны для joint_states и отладки.
  double left_wheel_position{0.0};
  double right_wheel_position{0.0};
};

// Интегратор держит минимальное состояние дифференциального привода без привязки к ROS 2 API.
class OdometryIntegrator
{
public:
  OdometryIntegrator(double wheel_radius, double wheel_base);

  // Полностью обнуляет накопленную одометрию.
  void Reset();

  // Выполняет один шаг интегрирования по скоростям колёс за интервал dt_sec.
  OdometryState Step(double left_rad_per_sec, double right_rad_per_sec, double dt_sec);

  // Возвращает текущее внутреннее состояние без дополнительного пересчёта.
  const OdometryState & state() const;

private:
  double wheel_radius_{0.0};
  double wheel_base_{0.0};
  OdometryState state_;
};

}  // namespace patrolbot_base

#endif  // PATROLBOT_BASE__ODOMETRY_INTEGRATOR_HPP_
