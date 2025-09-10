/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "node.hpp"
#include "rko_lio/core/profiler.hpp"
#include "rko_lio/ros_utils/rosbag_utils.hpp"
// other
#include <std_msgs/msg/float32_multi_array.hpp>

namespace {
template <typename T>
std::shared_ptr<T> deserialize_next_msg(const rclcpp::SerializedMessage& serialized_msg) {
  const auto msg = std::make_shared<T>();
  const rclcpp::Serialization<T> serializer;
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
};
} // namespace

namespace rko_lio::ros {
class OfflineNode : public Node {
public:
  std::unique_ptr<ros_utils::BufferableBag> bag;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_lidar_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bag_progress_publisher;
  float total_bag_msgs = 0;
  float processed_bag_msgs = 0;

  explicit OfflineNode(const rclcpp::NodeOptions& options) : Node("rko_lio_offline_node", options) {
    max_lidar_buffer_size = 100;
    // bag reading
    const tf2::Duration skip_to_time = tf2::durationFromSec(node->declare_parameter<double>("skip_to_time", 0.0));
    bag = std::make_unique<ros_utils::BufferableBag>(node->declare_parameter<std::string>("bag_path"),
                                                     std::make_shared<ros_utils::BufferableBag::TFBridge>(node),
                                                     std::vector<std::string>{imu_topic, lidar_topic}, skip_to_time);
    total_bag_msgs = bag->message_count();
    bag_progress_publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("/rko_lio/bag_progress", 10);
    if (debug) {
      const rclcpp::QoS publisher_qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
      raw_imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/rko_lio/raw_imu", publisher_qos);
      raw_lidar_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rko_lio/raw_lidar", publisher_qos);
    }
    RCLCPP_INFO_STREAM(node->get_logger(),
                       (debug ? " Debug: Publishing compensated IMU acceleration to /rko_lio/body_accel. Debug: "
                                "Re-publishing IMU data to " +
                                    imu_topic + " and lidar data to " + lidar_topic + "."
                              : ""));
  }

  void run() {
    while (rclcpp::ok() && !bag->finished()) {
      {
        if (lidar_buffer.size() >= 0.9 * max_lidar_buffer_size) {
          RCLCPP_WARN_STREAM_ONCE(node->get_logger(),
                                  "lidar buffer size: " << lidar_buffer.size()
                                                        << ", max_lidar_buffer_size: " << max_lidar_buffer_size
                                                        << ", throttling the bag reading thread\n");
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
      }
      const rosbag2_storage::SerializedBagMessage serialized_bag_msg = bag->PopNextMessage();
      const auto& topic_name = serialized_bag_msg.topic_name;
      const rclcpp::SerializedMessage serialized_msg(*serialized_bag_msg.serialized_data);
      // publish the bag progress now, because we have a continue inside the if elses
      processed_bag_msgs++;
      if (total_bag_msgs > 0) {
        const float percent_complete = 100.0F * processed_bag_msgs / total_bag_msgs;
        static const auto start_time = std::chrono::steady_clock::now();
        const auto now = std::chrono::steady_clock::now();
        const float elapsed_seconds = std::chrono::duration<float>(now - start_time).count();
        const float avg_time_per_msg = (processed_bag_msgs > 0) ? elapsed_seconds / processed_bag_msgs : 0.0F;
        const float seconds_remaining = avg_time_per_msg * (total_bag_msgs - processed_bag_msgs);
        std_msgs::msg::Float32MultiArray progress_msg;
        progress_msg.data.resize(2);
        progress_msg.data[0] = percent_complete;
        progress_msg.data[1] = seconds_remaining;
        bag_progress_publisher->publish(progress_msg);
      }
      // check the topic and call the appropriate callback
      if (topic_name == imu_topic) {
        const auto& imu_msg = deserialize_next_msg<sensor_msgs::msg::Imu>(serialized_msg);
        imu_callback(imu_msg);
        if (!debug) {
          continue;
        }
        raw_imu_publisher->publish(*imu_msg);
      } else if (topic_name == lidar_topic) {
        const auto& lidar_msg = deserialize_next_msg<sensor_msgs::msg::PointCloud2>(serialized_msg);
        lidar_callback(lidar_msg);
        if (!debug) {
          continue;
        }
        raw_lidar_publisher->publish(*lidar_msg);
      }
    }
    while (true) {
      // even if the bag finishes, we need to wait on the registration thread (buffers) to finish (empty)
      std::lock_guard lock(buffer_mutex);
      if (imu_buffer.empty() || lidar_buffer.empty()) {
        break;
      }
    }
  }
};
} // namespace rko_lio::ros

int main(int argc, char** argv) {
  const rko_lio::core::Timer timer("RKO LIO Offline Node");
  rclcpp::init(argc, argv);
  auto offline_node = rko_lio::ros::OfflineNode(rclcpp::NodeOptions());
  try {
    offline_node.run();
  } catch (const std::runtime_error& error) {
    RCLCPP_WARN_STREAM(
        rclcpp::get_logger("OfflineNode"),
        "Encountered runtime_error. Still attempting to dump trajectory to disk. The error was: " << error.what());
  }
  offline_node.lio->dump_results_to_disk(offline_node.results_dir, offline_node.run_name);
  rclcpp::shutdown();
  return 0;
}
