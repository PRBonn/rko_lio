// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// copied and modified from kinematic icp
#include "rosbag_utils.hpp"
// ROS
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/version.h>
#include <rosbag2_storage/bag_metadata.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
// stl
#include <algorithm>

namespace {
inline auto GetTimestampsFromRosbagSerializedMsg(const rosbag2_storage::SerializedBagMessage& msg) {
#if RCLCPP_VERSION_GTE(22, 0, 0)
  return std::chrono::nanoseconds(msg.recv_timestamp);
#else
  return std::chrono::nanoseconds(msg.time_stamp);
#endif
}
} // namespace

namespace rko_lio::ros_utils {
// TFBridge----------------------------------------------------------------------------------------
BufferableBag::TFBridge::TFBridge(rclcpp::Node::SharedPtr node) {
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
}

void BufferableBag::TFBridge::ProcessTFMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) const {
  tf2_msgs::msg::TFMessage tf_message;
  rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  serializer.deserialize_message(&serialized_msg, &tf_message);
  // Broadcast tranforms to /tf and /tf_static topics
  for (auto& transform : tf_message.transforms) {
    if (msg->topic_name == "/tf_static") {
      tf_static_broadcaster->sendTransform(transform);
    } else {
      tf_broadcaster->sendTransform(transform);
    }
  }
}

// BufferableBag-----------------------------------------------------------------------------------
BufferableBag::BufferableBag(const std::string& bag_path,
                             const std::shared_ptr<TFBridge> tf_bridge,
                             const std::vector<std::string>& topics,
                             const tf2::Duration seek,
                             const std::chrono::seconds buffer_size)
    : tf_bridge_(tf_bridge),
      bag_reader_(std::make_unique<rosbag2_cpp::Reader>()),
      buffer_size_(buffer_size),
      topics_(topics) {
  publish_tf_static(bag_path);
  bag_reader_->open(bag_path);
  bag_reader_->seek(seek.count());
  bag_reader_->set_filter(rosbag2_storage::StorageFilter{topics_});
  message_count_ = [&]() {
    size_t message_count = 0;
    const auto& metadata = bag_reader_->get_metadata();
    const auto topic_info = metadata.topics_with_message_count;
    // iterate over all topics
    for (const auto& topic : topics_) {
      const auto it = std::find_if(topic_info.cbegin(), topic_info.cend(),
                                   [&](const auto& info) { return info.topic_metadata.name == topic; });
      if (it != topic_info.end()) {
        message_count += it->message_count;
      }
    }
    return message_count;
  }();
  std::cout << "Bag reader initialized with total message count: " << message_count_ << '\n';
  BufferMessages();
}

void BufferableBag::publish_tf_static(const std::string& bag_path) {
  std::cout << "Opening the bag first to publish all the tf_static messages\n";
  rosbag2_cpp::Reader tf_reader;
  tf_reader.open(bag_path);
  tf_reader.set_filter(rosbag2_storage::StorageFilter{{"/tf_static"}});
  while (tf_reader.has_next()) {
    const auto msg = tf_reader.read_next();
    tf_bridge_->ProcessTFMessage(msg);
  }
  tf_reader.close();
  std::cout << "tf_static published, if any. Closing the bag...\n";
}

bool BufferableBag::finished() const { return !bag_reader_->has_next() && buffer_.empty(); };
void BufferableBag::close() const { bag_reader_->close(); }

size_t BufferableBag::message_count() const { return message_count_; }

void BufferableBag::BufferMessages() {
  auto buffer_is_filled = [&]() -> bool {
    if (buffer_.empty()) {
      return false;
    }
    const auto first_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.front());
    const auto last_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.back());
    return (last_stamp - first_stamp) > buffer_size_;
  };

  // Advance reading one message until the buffer is filled or we finish the bagfile
  while (!buffer_is_filled() && bag_reader_->has_next()) {
    // Fetch next message from bagfile, could be anything
    const auto msg = bag_reader_->read_next();
    // If the msg is TFMessage, fill the tf_buffer and broadcast the transformation and don't
    // populate the buffered_messages_ as we already processed it
    if (msg->topic_name == "/tf") {
      tf_bridge_->ProcessTFMessage(msg);
    } else if (std::find(topics_.cbegin(), topics_.cend(), msg->topic_name) != topics_.end()) {
      // If the msg is not a TFMessage and matches any topic in topics_, push it to the internal
      // buffer
      buffer_.push(*msg);
    }
  }
}

rosbag2_storage::SerializedBagMessage BufferableBag::PopNextMessage() {
  const rosbag2_storage::SerializedBagMessage msg = buffer_.front();
  buffer_.pop();
  if (bag_reader_->has_next()) {
    BufferMessages();
  }
  return msg;
}
} // namespace rko_lio::ros_utils
