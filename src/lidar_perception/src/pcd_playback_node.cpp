// Copyright 2026 ros2_lidar_perception contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace lidar_perception
{

class PcdPlaybackNode final : public rclcpp::Node
{
public:
  PcdPlaybackNode()
  : Node("pcd_playback")
  {
    const auto path = declare_parameter<std::string>("path", "");
    topic_ = declare_parameter<std::string>("topic", "/points_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    loop_ = declare_parameter<bool>("loop", true);
    if (path.empty()) {throw std::invalid_argument("path must name a PCD file or directory");}
    if (rate_hz_ <= 0.0) {throw std::invalid_argument("rate_hz must be positive");}
    const std::filesystem::path input(path);
    if (std::filesystem::is_regular_file(input) && input.extension() == ".pcd") {
      files_.push_back(input);
    } else if (std::filesystem::is_directory(input)) {
      for (const auto & entry : std::filesystem::directory_iterator(input)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
          files_.push_back(entry.path());
        }
      }
      std::sort(files_.begin(), files_.end());
    }
    if (files_.empty()) {throw std::runtime_error("no .pcd files found at: " + path);}
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 /
        rate_hz_)),
      std::bind(&PcdPlaybackNode::publishNext, this));
    RCLCPP_INFO(get_logger(), "Loaded %zu PCD path(s) from %s", files_.size(), path.c_str());
  }

private:
  void publishNext()
  {
    if (index_ >= files_.size()) {
      if (!loop_) {timer_->cancel(); return;}
      index_ = 0;
    }
    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(files_[index_].string(), pcl_cloud) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load %s", files_[index_].c_str());
      ++index_;
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl_conversions::fromPCL(pcl_cloud, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    publisher_->publish(message);
    ++index_;
  }

  std::vector<std::filesystem::path> files_;
  std::size_t index_{0};
  std::string topic_;
  std::string frame_id_;
  double rate_hz_{10.0};
  bool loop_{true};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lidar_perception::PcdPlaybackNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_playback"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
