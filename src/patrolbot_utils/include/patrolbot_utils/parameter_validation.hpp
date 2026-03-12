#ifndef PATROLBOT_UTILS__PARAMETER_VALIDATION_HPP_
#define PATROLBOT_UTILS__PARAMETER_VALIDATION_HPP_

#include <string>
#include <vector>

namespace patrolbot_utils
{

struct ValidationIssue
{
  // Имя поля или параметра, в котором найдена проблема.
  std::string field;

  // Короткое описание нарушения.
  std::string message;
};

using ValidationIssues = std::vector<ValidationIssue>;

// Проверяет, что значение строго положительно.
void RequirePositive(const std::string & field, double value, ValidationIssues * issues);

// Проверяет, что целочисленное значение строго положительно.
void RequirePositive(const std::string & field, int value, ValidationIssues * issues);

// Проверяет, что значение не отрицательное.
void RequireNonNegative(const std::string & field, double value, ValidationIssues * issues);

// Проверяет, что строковое значение не пустое.
void RequireNotEmpty(const std::string & field, const std::string & value, ValidationIssues * issues);

// Преобразует список ошибок в удобочитаемую строку для исключений и логов.
std::string FormatIssues(const ValidationIssues & issues);

// Возвращает true, если в отчёте есть хотя бы одна ошибка.
bool HasIssues(const ValidationIssues & issues);

}  // namespace patrolbot_utils

#endif  // PATROLBOT_UTILS__PARAMETER_VALIDATION_HPP_
