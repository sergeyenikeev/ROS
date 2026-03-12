#include "patrolbot_base/odometry_integrator.hpp"

#include <cmath>

namespace patrolbot_base
{

OdometryIntegrator::OdometryIntegrator(double wheel_radius, double wheel_base)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base)
{
}

void OdometryIntegrator::Reset()
{
  state_ = OdometryState{};
}

OdometryState OdometryIntegrator::Step(
  double left_rad_per_sec,
  double right_rad_per_sec,
  double dt_sec)
{
  const double left_linear = left_rad_per_sec * wheel_radius_;
  const double right_linear = right_rad_per_sec * wheel_radius_;

  state_.linear_velocity = (left_linear + right_linear) * 0.5;
  state_.angular_velocity = (right_linear - left_linear) / wheel_base_;

  const double delta_yaw = state_.angular_velocity * dt_sec;
  const double delta_distance = state_.linear_velocity * dt_sec;
  const double heading_mid = state_.yaw + delta_yaw * 0.5;

  state_.x += delta_distance * std::cos(heading_mid);
  state_.y += delta_distance * std::sin(heading_mid);
  state_.yaw += delta_yaw;

  state_.left_wheel_position += left_rad_per_sec * dt_sec;
  state_.right_wheel_position += right_rad_per_sec * dt_sec;

  return state_;
}

const OdometryState & OdometryIntegrator::state() const
{
  return state_;
}

}  // namespace patrolbot_base
