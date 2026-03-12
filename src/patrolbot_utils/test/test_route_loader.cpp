#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <string>

#include "patrolbot_utils/route_loader.hpp"

namespace
{

std::filesystem::path WriteMissionFile(const std::string & file_name, const std::string & content)
{
  const auto path = std::filesystem::temp_directory_path() / file_name;
  std::ofstream stream(path);
  stream << content;
  stream.close();
  return path;
}

}  // namespace

TEST(RouteLoaderTest, LoadsValidMissionWithDefaults)
{
  const auto path = WriteMissionFile(
    "patrolbot_valid_mission.yaml",
    R"(mission:
  name: test_patrol
  frame_id: map
  autostart: true
  loop_forever: false
  continue_on_error: false
  default_timeout_sec: 50.0
  default_retries: 2
  retry_backoff_sec: 3.0
  waypoints:
    - name: wp1
      x: 1.0
      y: 2.0
      yaw_deg: 90.0
)");

  const auto mission = patrolbot_utils::LoadMissionConfig(path.string());

  EXPECT_EQ(mission.name, "test_patrol");
  ASSERT_EQ(mission.waypoints.size(), 1U);
  EXPECT_EQ(mission.waypoints.front().name, "wp1");
  EXPECT_NEAR(mission.waypoints.front().yaw_rad, 1.57079632679, 1e-6);
  EXPECT_DOUBLE_EQ(mission.waypoints.front().timeout_sec, 50.0);
  EXPECT_EQ(mission.waypoints.front().retries, 2);

  std::filesystem::remove(path);
}

TEST(RouteLoaderTest, RejectsNegativeRetries)
{
  const auto path = WriteMissionFile(
    "patrolbot_invalid_mission.yaml",
    R"(mission:
  name: invalid_patrol
  frame_id: map
  default_timeout_sec: 20.0
  default_retries: 1
  retry_backoff_sec: 2.0
  waypoints:
    - name: wp1
      x: 0.0
      y: 0.0
      yaw_deg: 0.0
      retries: -1
)");

  EXPECT_THROW(
    patrolbot_utils::LoadMissionConfig(path.string()),
    std::runtime_error);

  std::filesystem::remove(path);
}
