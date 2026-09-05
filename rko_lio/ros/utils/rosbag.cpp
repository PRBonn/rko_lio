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
#include "rosbag.hpp"
// ROS
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/version.h>
#include <rosbag2_storage/bag_metadata.hpp>
// other
#include <spdlog/spdlog.h>
// stl
#include <algorithm>
#include <functional>
#include <utility>

namespace {
inline auto GetTimestampsFromRosbagSerializedMsg(const rosbag2_storage::SerializedBagMessage& msg) {
#if RCLCPP_VERSION_GTE(22, 0, 0)
  return std::chrono::nanoseconds(msg.recv_timestamp);
#else
  return std::chrono::nanoseconds(msg.time_stamp);
#endif
}
} // namespace

namespace rko_lio::ros::utils {
BufferableBag::BufferableBag(const std::string& bag_path,
                             const std::vector<std::string>& topics,
                             std::shared_ptr<tf2::BufferCore> tf_buffer,
                             const tf2::Duration skip_from_start,
                             const std::chrono::seconds buffer_size)
    : tf_buffer_(std::move(tf_buffer)),
      bag_reader_(std::make_unique<rosbag2_cpp::Reader>()),
      buffer_size_(buffer_size),
      topics_(topics) {
  load_tf_static(bag_path);
  bag_reader_->open(bag_path);
  std::vector<std::string> filter_topics = topics_;
  filter_topics.emplace_back("/tf");
  bag_reader_->set_filter(rosbag2_storage::StorageFilter{.topics = filter_topics});
  const auto& metadata = bag_reader_->get_metadata();
  const auto bag_start = std::chrono::duration_cast<tf2::Duration>(metadata.starting_time.time_since_epoch());
  bag_reader_->seek((bag_start + skip_from_start).count());
  message_count_ = std::invoke([&] {
    size_t message_count = 0;
    const auto topic_info = metadata.topics_with_message_count;
    for (const auto& topic : topics_) {
      const auto it = std::find_if(topic_info.cbegin(), topic_info.cend(),
                                   [&](const auto& info) { return info.topic_metadata.name == topic; });
      if (it != topic_info.end()) {
        message_count += it->message_count;
      }
    }
    return message_count;
  });
  spdlog::info("Bag reader initialized with total message count: {}", message_count_);
  BufferMessages();
}

void BufferableBag::load_tf_static(const std::string& bag_path) {
  spdlog::info("Loading /tf_static into the tf buffer");
  rosbag2_cpp::Reader tf_reader;
  tf_reader.open(bag_path);
  tf_reader.set_filter(rosbag2_storage::StorageFilter{.topics = {"/tf_static"}});
  while (tf_reader.has_next()) {
    apply_tf_message(tf_reader.read_next());
  }
  tf_reader.close();
}

bool BufferableBag::finished() const { return !bag_reader_->has_next() && buffer_.empty(); };
void BufferableBag::close() const { bag_reader_->close(); }

size_t BufferableBag::message_count() const { return message_count_; }

void BufferableBag::BufferMessages() {
  const auto buffer_is_filled = [&]() -> bool {
    if (buffer_.empty()) {
      return false;
    }
    const auto first_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.front());
    const auto last_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.back());
    return (last_stamp - first_stamp) > buffer_size_;
  };

  while (!buffer_is_filled() && bag_reader_->has_next()) {
    const auto msg = bag_reader_->read_next();
    if (msg->topic_name == "/tf") {
      apply_tf_message(msg);
    } else if (std::find(topics_.cbegin(), topics_.cend(), msg->topic_name) != topics_.end()) {
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

void BufferableBag::apply_tf_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) const {
  tf2_msgs::msg::TFMessage tf_message;
  const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  serializer_.deserialize_message(&serialized_msg, &tf_message);
  const bool is_static = msg->topic_name == "/tf_static";
  for (const auto& transform : tf_message.transforms) {
    tf_buffer_->setTransform(transform, "bag", is_static);
  }
}
} // namespace rko_lio::ros::utils
