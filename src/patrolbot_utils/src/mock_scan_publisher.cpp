#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "patrolbot_utils/logging_utils.hpp"
#include "patrolbot_utils/parameter_validation.hpp"

namespace patrolbot_utils
{

class MockScanPublisher : public rclcpp::Node
{
public:
  MockScanPublisher()
  : Node("mock_scan_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 8.0);
    angle_min_ = declare_parameter<double>("angle_min", -3.14159265359);
    angle_max_ = declare_parameter<double>("angle_max", 3.14159265359);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.01745329251);
    range_min_ = declare_parameter<double>("range_min", 0.12);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    default_range_ = declare_parameter<double>("default_range", 3.5);
    verbose_logging_ = declare_parameter<bool>("verbose_logging", false);

    ValidationIssues issues;
    RequireNotEmpty("frame_id", frame_id_, &issues);
    RequireNotEmpty("scan_topic", scan_topic_, &issues);
    RequirePositive("publish_rate_hz", publish_rate_hz_, &issues);
    RequirePositive("angle_increment", angle_increment_, &issues);
    RequirePositive("range_min", range_min_, &issues);
    RequirePositive("range_max", range_max_, &issues);
    RequirePositive("default_range", default_range_, &issues);

    if (angle_max_ <= angle_min_) {
      issues.push_back({"angle_max", "должно быть больше angle_min"});
    }

    if (range_max_ <= range_min_) {
      issues.push_back({"range_max", "должно быть больше range_min"});
    }

    if (HasIssues(issues)) {
      const auto text = FormatIssues(issues);
      RCLCPP_FATAL(
        get_logger(),
        "%s",
        MakeLogMessage("ошибка", "некорректные параметры mock_scan_publisher: " + text).c_str());
      throw std::runtime_error(text);
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&MockScanPublisher::PublishScan, this));

    RCLCPP_INFO(
      get_logger(),
      "%s",
      MakeLogMessage(
        "узел",
        "mock_scan_publisher запущен и публикует тестовый LaserScan в " + scan_topic_).c_str());
  }

private:
  void PublishScan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(1.0 / publish_rate_hz_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    const auto beam_count = static_cast<std::size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
    scan.ranges.resize(beam_count, static_cast<float>(default_range_));
    scan.intensities.resize(beam_count, 1.0F);

    // Небольшая синтетическая структура помогает отлаживать SLAM без реального LiDAR.
    for (std::size_t index = 0; index < beam_count; ++index) {
      const double angle = angle_min_ + static_cast<double>(index) * angle_increment_;
      const double synthetic_obstacle =
        default_range_ - 0.4 * std::sin(angle * 3.0) - 0.2 * std::cos(angle * 5.0);
      scan.ranges[index] = static_cast<float>(
        std::clamp(synthetic_obstacle, range_min_ + 0.05, range_max_ - 0.05));
    }

    publisher_->publish(scan);

    if (verbose_logging_) {
      RCLCPP_DEBUG(
        get_logger(),
        "%s",
        MakeLogMessage("датчик", "опубликован mock LaserScan").c_str());
    }
  }

  std::string frame_id_;
  std::string scan_topic_;
  double publish_rate_hz_{0.0};
  double angle_min_{0.0};
  double angle_max_{0.0};
  double angle_increment_{0.0};
  double range_min_{0.0};
  double range_max_{0.0};
  double default_range_{0.0};
  bool verbose_logging_{false};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace patrolbot_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<patrolbot_utils::MockScanPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
