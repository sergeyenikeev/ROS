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
  // Helper централизует добавление ошибок и аккуратно обрабатывает сценарий,
  // когда вызывающему коду список ошибок не нужен.
  if (issues == nullptr) {
    return;
  }

  issues->push_back(ValidationIssue{field, message});
}

}  // namespace

void RequirePositive(const std::string & field, double value, ValidationIssues * issues)
{
  // Строго положительные значения требуются для физических величин вроде
  // радиуса колеса, частоты цикла и таймаутов.
  if (value <= 0.0) {
    AddIssue(field, "должно быть больше нуля", issues);
  }
}

void RequirePositive(const std::string & field, int value, ValidationIssues * issues)
{
  // Отдельная перегрузка для целочисленных параметров избавляет вызывающий код
  // от лишних преобразований типов.
  if (value <= 0) {
    AddIssue(field, "должно быть больше нуля", issues);
  }
}

void RequireNonNegative(const std::string & field, double value, ValidationIssues * issues)
{
  // Неотрицательные значения применяются там, где ноль допустим, например для
  // некоторых счётчиков или специальных порогов.
  if (value < 0.0) {
    AddIssue(field, "не должно быть отрицательным", issues);
  }
}

void RequireNotEmpty(
  const std::string & field,
  const std::string & value,
  ValidationIssues * issues)
{
  // Пустые строки особенно опасны для имён frame, topic и файловых путей.
  if (value.empty()) {
    AddIssue(field, "не должно быть пустым", issues);
  }
}

std::string FormatIssues(const ValidationIssues & issues)
{
  // Все ошибки собираются в одну строку, чтобы ими было удобно бросать
  // исключение и печатать их в ROS-лог.
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
