#include "patrolbot_utils/parameter_validation.hpp"

#include <sstream>

namespace patrolbot_utils
{

namespace
{

void AddIssue(
  const std::string & field,
  const std::string & message,
  ValidationIssues * issues)
{
  if (issues == nullptr) {
    return;
  }

  issues->push_back(ValidationIssue{field, message});
}

}  // namespace

void RequirePositive(const std::string & field, double value, ValidationIssues * issues)
{
  if (value <= 0.0) {
    AddIssue(field, "должно быть больше нуля", issues);
  }
}

void RequirePositive(const std::string & field, int value, ValidationIssues * issues)
{
  if (value <= 0) {
    AddIssue(field, "должно быть больше нуля", issues);
  }
}

void RequireNonNegative(const std::string & field, double value, ValidationIssues * issues)
{
  if (value < 0.0) {
    AddIssue(field, "не должно быть отрицательным", issues);
  }
}

void RequireNotEmpty(
  const std::string & field,
  const std::string & value,
  ValidationIssues * issues)
{
  if (value.empty()) {
    AddIssue(field, "не должно быть пустым", issues);
  }
}

std::string FormatIssues(const ValidationIssues & issues)
{
  std::ostringstream stream;

  for (std::size_t index = 0; index < issues.size(); ++index) {
    stream << issues[index].field << ": " << issues[index].message;
    if (index + 1U < issues.size()) {
      stream << "; ";
    }
  }

  return stream.str();
}

bool HasIssues(const ValidationIssues & issues)
{
  return !issues.empty();
}

}  // namespace patrolbot_utils
