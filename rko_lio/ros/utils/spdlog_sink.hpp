#pragma once
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <string>

namespace rko_lio::ros::utils {

class RclcppSink : public spdlog::sinks::base_sink<std::mutex> {
public:
  explicit RclcppSink(rclcpp::Logger logger) : logger_(std::move(logger)) {}

protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    const std::string text(msg.payload.begin(), msg.payload.end());
    switch (msg.level) {
    case spdlog::level::trace:
    case spdlog::level::debug:
      RCLCPP_DEBUG_STREAM(logger_, text);
      break;
    case spdlog::level::warn:
      RCLCPP_WARN_STREAM(logger_, text);
      break;
    case spdlog::level::err:
      RCLCPP_ERROR_STREAM(logger_, text);
      break;
    case spdlog::level::critical:
      RCLCPP_FATAL_STREAM(logger_, text);
      break;
    case spdlog::level::info:
    case spdlog::level::off:
    case spdlog::level::n_levels:
    default:
      RCLCPP_INFO_STREAM(logger_, text);
      break;
    }
  }
  void flush_() override {}

private:
  rclcpp::Logger logger_;
};

} // namespace rko_lio::ros::utils
