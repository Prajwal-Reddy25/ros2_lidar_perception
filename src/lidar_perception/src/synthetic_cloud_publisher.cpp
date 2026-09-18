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
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace lidar_perception
{

class SyntheticCloudPublisher final : public rclcpp::Node
{
public:
  SyntheticCloudPublisher()
  : Node("synthetic_cloud_publisher")
  {
    topic_ = declare_parameter<std::string>("topic", "/points_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    seed_ = declare_parameter<int>("seed", 42);
    ground_points_ = declare_parameter<int>("ground_points", 12000);
    points_per_object_ = declare_parameter<int>("points_per_object", 900);
    noise_stddev_ = declare_parameter<double>("noise_stddev", 0.02);
    moving_objects_ = declare_parameter<bool>("moving_objects", true);
    if (rate_hz_ <= 0.0 || ground_points_ < 0 || points_per_object_ < 1 || noise_stddev_ < 0.0) {
      throw std::invalid_argument("invalid synthetic point-cloud parameters");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SyntheticCloudPublisher::publish, this));
    RCLCPP_INFO(get_logger(), "Publishing deterministic synthetic clouds on %s at %.1f Hz",
        topic_.c_str(), rate_hz_);
  }

private:
  void addCuboid(
    pcl::PointCloud<pcl::PointXYZI> & cloud, std::mt19937 & generator,
    const float cx, const float cy, const float sx, const float sy, const float sz, const float yaw)
  {
    std::uniform_real_distribution<float> x_distribution(-0.5F * sx, 0.5F * sx);
    std::uniform_real_distribution<float> y_distribution(-0.5F * sy, 0.5F * sy);
    std::uniform_real_distribution<float> z_distribution(0.0F, sz);
    std::normal_distribution<float> noise(0.0F, static_cast<float>(noise_stddev_));
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    for (int i = 0; i < points_per_object_; ++i) {
      const float local_x = x_distribution(generator);
      const float local_y = y_distribution(generator);
      pcl::PointXYZI point;
      point.x = cx + cosine * local_x - sine * local_y + noise(generator);
      point.y = cy + sine * local_x + cosine * local_y + noise(generator);
      point.z = z_distribution(generator) + noise(generator);
      point.intensity = 80.0F + static_cast<float>(i % 40);
      cloud.push_back(point);
    }
  }

  void publish()
  {
    std::mt19937 generator(static_cast<std::uint32_t>(seed_ + frame_index_));
    std::uniform_real_distribution<float> ground_x(-30.0F, 55.0F);
    std::uniform_real_distribution<float> ground_y(-14.0F, 14.0F);
    std::normal_distribution<float> ground_z(0.0F, static_cast<float>(noise_stddev_));
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(static_cast<std::size_t>(ground_points_ + 3 * points_per_object_ + 50));
    for (int i = 0; i < ground_points_; ++i) {
      pcl::PointXYZI point;
      point.x = ground_x(generator);
      point.y = ground_y(generator);
      point.z = ground_z(generator);
      point.intensity = 10.0F;
      cloud.push_back(point);
    }
    const float time = static_cast<float>(frame_index_ / rate_hz_);
    const float moving_x = moving_objects_ ? 9.0F + 1.5F * time : 9.0F;
    addCuboid(cloud, generator, moving_x, 2.5F, 4.2F, 1.9F, 1.6F, 0.18F);
    addCuboid(cloud, generator, 20.0F, -4.0F, 0.8F, 0.8F, 1.75F, -0.3F);
    addCuboid(cloud, generator, 32.0F, 5.5F, 6.0F, 2.4F, 2.8F, 0.05F);
    for (int i = 0; i < 50; ++i) {
      pcl::PointXYZI outlier;
      outlier.x = ground_x(generator);
      outlier.y = ground_y(generator);
      outlier.z = 1.0F + 3.0F * static_cast<float>(i) / 49.0F;
      outlier.intensity = 1.0F;
      cloud.push_back(outlier);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    publisher_->publish(message);
    ++frame_index_;
  }

  std::string topic_;
  std::string frame_id_;
  double rate_hz_{10.0};
  int seed_{42};
  int ground_points_{12000};
  int points_per_object_{900};
  double noise_stddev_{0.02};
  bool moving_objects_{true};
  std::uint64_t frame_index_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_perception::SyntheticCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
