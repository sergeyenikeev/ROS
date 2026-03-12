#include "patrolbot_base/odometry_integrator.hpp"

#include <cmath>

namespace patrolbot_base
{

// Интегратор хранит только геометрию базы и текущее состояние позы. Он не знает
// ничего о ROS-сообщениях, драйверах и таймерах, поэтому хорошо подходит для
// изолированного unit-тестирования.
OdometryIntegrator::OdometryIntegrator(double wheel_radius, double wheel_base)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base)
{
}

void OdometryIntegrator::Reset()
{
  // Полный сброс удобен для тестов и для сценариев, когда нужно обнулить
  // накопленную одометрию без пересоздания объекта.
  state_ = OdometryState{};
}

OdometryState OdometryIntegrator::Step(
  double left_rad_per_sec,
  double right_rad_per_sec,
  double dt_sec)
{
  // Скорости колёс переводятся из угловых в линейные по известному радиусу.
  const double left_linear = left_rad_per_sec * wheel_radius_;
  const double right_linear = right_rad_per_sec * wheel_radius_;

  // Для дифференциальной базы скорость корпуса выражается через среднюю
  // линейную скорость колёс и разность их скоростей.
  state_.linear_velocity = (left_linear + right_linear) * 0.5;
  state_.angular_velocity = (right_linear - left_linear) / wheel_base_;

  // Интегрирование выполняется по midpoint-схеме: ориентация для обновления x/y
  // берётся посередине шага. Это немного точнее простого Эйлера на поворотах.
  const double delta_yaw = state_.angular_velocity * dt_sec;
  const double delta_distance = state_.linear_velocity * dt_sec;
  const double heading_mid = state_.yaw + delta_yaw * 0.5;

  state_.x += delta_distance * std::cos(heading_mid);
  state_.y += delta_distance * std::sin(heading_mid);
  state_.yaw += delta_yaw;

  // Положения колёс накапливаются отдельно для публикации в joint_states.
  state_.left_wheel_position += left_rad_per_sec * dt_sec;
  state_.right_wheel_position += right_rad_per_sec * dt_sec;

  return state_;
}

const OdometryState & OdometryIntegrator::state() const
{
  return state_;
}

}  // namespace patrolbot_base
