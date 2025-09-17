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
}

using BagProgressPublisher = rclcpp::Publisher<std_msgs::msg::Float32MultiArray>;
void publish_bag_progress(const BagProgressPublisher::SharedPtr& publisher,
                          const size_t processed_bag_msgs,
                          const size_t total_bag_msgs) {
  if (total_bag_msgs == 0) {
    return;
  }
  static const auto start_time = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now();
  const float elapsed_seconds = std::chrono::duration<float>(now - start_time).count();

  const float percent_complete = 100.0F * processed_bag_msgs / total_bag_msgs;
  const float avg_time_per_msg = (processed_bag_msgs > 0) ? elapsed_seconds / processed_bag_msgs : 0.0F;
  const float seconds_remaining = avg_time_per_msg * (total_bag_msgs - processed_bag_msgs);

  std_msgs::msg::Float32MultiArray progress_msg;
  progress_msg.data.resize(2);
  progress_msg.data[0] = percent_complete;
  progress_msg.data[1] = seconds_remaining;

  publisher->publish(progress_msg);
}
} // namespace

namespace rko_lio::ros {
class OfflineNode : public Node {
public:
  std::unique_ptr<ros_utils::BufferableBag> bag;

  BagProgressPublisher::SharedPtr bag_progress_publisher;

  float total_bag_msgs = 0;
  float processed_bag_msgs = 0;

  explicit OfflineNode(const rclcpp::NodeOptions& options) : Node("rko_lio_offline_node", options) {
    // increase the lidar buffer limit because we're offline
    max_lidar_buffer_size = 100;
    // bag reading
    const tf2::Duration skip_to_time = tf2::durationFromSec(node->declare_parameter<double>("skip_to_time", 0.0));
    bag = std::make_unique<ros_utils::BufferableBag>(node->declare_parameter<std::string>("bag_path"),
                                                     std::make_shared<ros_utils::BufferableBag::TFBridge>(node),
                                                     std::vector<std::string>{imu_topic, lidar_topic}, skip_to_time);
    total_bag_msgs = bag->message_count();
    bag_progress_publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("/rko_lio/bag_progress", 10);
  }

  void run() {
    while (rclcpp::ok() && !bag->finished()) {
      {
        if (lidar_buffer.size() >= 0.9 * max_lidar_buffer_size) {
          RCLCPP_WARN_STREAM_ONCE(node->get_logger(),
                                  "Lidar buffer size: " << lidar_buffer.size()
                                                        << ", max_lidar_buffer_size: " << max_lidar_buffer_size
                                                        << ", throttling the bag reading thread as it's too fast.\n");
          // this is a hack. can be improved
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
      }
      const rosbag2_storage::SerializedBagMessage serialized_bag_msg = bag->PopNextMessage();
      const auto& topic_name = serialized_bag_msg.topic_name;
      const rclcpp::SerializedMessage serialized_msg(*serialized_bag_msg.serialized_data);

      // check the topic and call the appropriate callback
      if (topic_name == imu_topic) {
        const auto& imu_msg = deserialize_next_msg<sensor_msgs::msg::Imu>(serialized_msg);
        imu_callback(imu_msg);
      } else if (topic_name == lidar_topic) {
        const auto& lidar_msg = deserialize_next_msg<sensor_msgs::msg::PointCloud2>(serialized_msg);
        lidar_callback(lidar_msg);
      }

      processed_bag_msgs++;
      publish_bag_progress(bag_progress_publisher, processed_bag_msgs, total_bag_msgs);
    }
    while (rclcpp::ok()) {
      {
        // even if the bag finishes, we need to wait on the registration buffers to empty
        std::lock_guard<std::mutex> lock(buffer_mutex);
        if (imu_buffer.empty() && lidar_buffer.empty()) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
