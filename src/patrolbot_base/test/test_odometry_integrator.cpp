#include "gtest/gtest.h"

#include "patrolbot_base/odometry_integrator.hpp"

TEST(OdometryIntegratorTest, StraightMotionIntegratesForwardDistance)
{
  // Этот тест проверяет базовый сценарий: оба колеса вращаются одинаково,
  // значит робот должен ехать строго вперёд без бокового смещения и поворота.
  patrolbot_base::OdometryIntegrator integrator(0.05, 0.24);

  const auto state = integrator.Step(10.0, 10.0, 1.0);

  EXPECT_NEAR(state.x, 0.5, 1e-6);
  EXPECT_NEAR(state.y, 0.0, 1e-6);
  EXPECT_NEAR(state.yaw, 0.0, 1e-6);
  EXPECT_NEAR(state.left_wheel_position, 10.0, 1e-6);
  EXPECT_NEAR(state.right_wheel_position, 10.0, 1e-6);
}

TEST(OdometryIntegratorTest, OppositeWheelVelocitiesRotateRobot)
{
  // Здесь проверяется разворот на месте: колёса вращаются в разные стороны,
  // поэтому линейная скорость корпуса должна быть нулевой, а угловая — ненулевой.
  patrolbot_base::OdometryIntegrator integrator(0.05, 0.24);

  const auto state = integrator.Step(-10.0, 10.0, 1.0);

  EXPECT_NEAR(state.x, 0.0, 1e-6);
  EXPECT_NEAR(state.y, 0.0, 1e-6);
  EXPECT_NEAR(state.yaw, 4.1666666667, 1e-6);
  EXPECT_NEAR(state.linear_velocity, 0.0, 1e-6);
  EXPECT_NEAR(state.angular_velocity, 4.1666666667, 1e-6);
}
