#include "gtest/gtest.h"

#include "patrolbot_utils/parameter_validation.hpp"

TEST(ParameterValidationTest, PositiveCheckAddsIssuesForInvalidValues)
{
  patrolbot_utils::ValidationIssues issues;
  patrolbot_utils::RequirePositive("wheel_radius", 0.0, &issues);
  patrolbot_utils::RequirePositive("cmd_timeout_ms", -1, &issues);

  ASSERT_EQ(issues.size(), 2U);
  EXPECT_TRUE(patrolbot_utils::HasIssues(issues));
  EXPECT_NE(patrolbot_utils::FormatIssues(issues).find("wheel_radius"), std::string::npos);
}

TEST(ParameterValidationTest, NonNegativeAndNotEmptyPassOnValidData)
{
  patrolbot_utils::ValidationIssues issues;
  patrolbot_utils::RequireNonNegative("timeout", 0.0, &issues);
  patrolbot_utils::RequireNotEmpty("driver_mode", "mock", &issues);

  EXPECT_FALSE(patrolbot_utils::HasIssues(issues));
  EXPECT_TRUE(patrolbot_utils::FormatIssues(issues).empty());
}
