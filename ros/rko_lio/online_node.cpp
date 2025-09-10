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

namespace rko_lio::ros {
class OnlineNode : public Node {
public:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
  rko_lio::core::Timer timer;

  OnlineNode(const OnlineNode&) = delete;
  OnlineNode(OnlineNode&&) = delete;
  OnlineNode& operator=(const OnlineNode&) = delete;
  OnlineNode& operator=(OnlineNode&&) = delete;

  explicit OnlineNode(const rclcpp::NodeOptions& options)
      : Node("rko_lio_online_node", options), timer("RKO LIO Online Node") {
    const auto qos_imu = rclcpp::SensorDataQoS().keep_last(100);
    const auto qos_lidar = rclcpp::SensorDataQoS().keep_last(10);

    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_imu, [this](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) { imu_callback(imu_msg); });

    lidar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, qos_lidar,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg) { lidar_callback(lidar_msg); });
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node->get_node_base_interface();
  }

  ~OnlineNode() { lio->dump_results_to_disk(results_dir, run_name); }
};
} // namespace rko_lio::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rko_lio::ros::OnlineNode)
