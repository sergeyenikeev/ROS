#include "gtest/gtest.h"

#include "patrolbot_mission_manager/mission_state_machine.hpp"

namespace
{

patrolbot_utils::MissionConfig MakeMission()
{
  patrolbot_utils::MissionConfig mission;
  mission.name = "test_patrol";
  mission.frame_id = "map";
  mission.default_timeout_sec = 10.0;
  mission.default_retries = 1;
  mission.retry_backoff_sec = 2.0;
  mission.waypoints = {
    patrolbot_utils::WaypointConfig{"wp1", 0.0, 0.0, 0.0, 10.0, 1},
    patrolbot_utils::WaypointConfig{"wp2", 1.0, 0.0, 0.0, 10.0, 0},
  };
  return mission;
}

}  // namespace

TEST(MissionStateMachineTest, WaitsForNav2AndDispatchesWhenServerAppears)
{
  patrolbot_mission_manager::MissionStateMachine machine;
  auto decision = machine.ActivateMission(MakeMission(), false);

  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kWaitForNav2);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kWaitingForNav2);

  decision = machine.HandleNav2Ready();
  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kDispatchGoal);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kNavigating);
}

TEST(MissionStateMachineTest, SchedulesRetryBeforeFinalSuccess)
{
  patrolbot_mission_manager::MissionStateMachine machine;
  machine.ActivateMission(MakeMission(), true);

  auto decision = machine.HandleGoalFailure("цель прервана");
  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kScheduleRetry);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kRetryWait);
  EXPECT_EQ(machine.snapshot().retry_count_for_current_waypoint, 1U);

  decision = machine.HandleRetryTimer();
  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kDispatchGoal);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kNavigating);
}

TEST(MissionStateMachineTest, FailureWithoutContinueOnErrorStopsMission)
{
  patrolbot_mission_manager::MissionStateMachine machine;
  auto mission = MakeMission();
  mission.waypoints.front().retries = 0;
  machine.ActivateMission(mission, true);

  const auto decision = machine.HandleGoalFailure("ошибка навигации");

  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kFailMission);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kFailed);
  EXPECT_FALSE(machine.snapshot().active);
}

TEST(MissionStateMachineTest, ContinueOnErrorMovesToNextWaypoint)
{
  patrolbot_mission_manager::MissionStateMachine machine;
  auto mission = MakeMission();
  mission.continue_on_error = true;
  mission.waypoints.front().retries = 0;
  machine.ActivateMission(mission, true);

  const auto decision = machine.HandleGoalFailure("ошибка навигации");

  EXPECT_EQ(decision.command, patrolbot_mission_manager::MissionCommand::kDispatchGoal);
  EXPECT_EQ(machine.snapshot().state, patrolbot_mission_manager::MissionState::kNavigating);
  EXPECT_EQ(machine.snapshot().current_waypoint_index, 1U);
  EXPECT_EQ(machine.snapshot().failed_waypoints, 1U);
}
