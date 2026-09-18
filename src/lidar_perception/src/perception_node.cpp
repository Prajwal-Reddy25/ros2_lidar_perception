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


#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lidar_perception/processor.hpp"
#include "lidar_perception/tracker.hpp"
#include "lidar_perception_msgs/msg/cluster.hpp"
#include "lidar_perception_msgs/msg/cluster_array.hpp"
#include "lidar_perception_msgs/msg/pipeline_metrics.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"


namespace lidar_perception
{

class PerceptionNode final : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("lidar_perception"), updater_(this)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/points_raw");
    queue_depth_ = declare_parameter<int>("queue_depth", 5);
    tracking_enabled_ = declare_parameter<bool>("tracking.enabled", true);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", true);
    if (input_topic_.empty()) {throw std::invalid_argument("input_topic must not be empty");}
    if (queue_depth_ < 1) {throw std::invalid_argument("queue_depth must be positive");}
    processor_ = std::make_unique<Processor>(readPipelineConfig());
    tracker_ = std::make_unique<MultiObjectTracker>(readTrackerConfig());

    const auto sensor_qos =
      rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(queue_depth_));
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&PerceptionNode::cloudCallback, this, std::placeholders::_1));
    filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered", sensor_qos);
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground", sensor_qos);
    nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/nonground",
        sensor_qos);
    cluster_publisher_ = create_publisher<lidar_perception_msgs::msg::ClusterArray>("~/clusters",
        10);
    detection_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    metrics_publisher_ = create_publisher<lidar_perception_msgs::msg::PipelineMetrics>("~/metrics",
        10);

    updater_.setHardwareID("software");
    updater_.add("pipeline", this, &PerceptionNode::diagnosticCallback);
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&PerceptionNode::parameterCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on %s; tracking=%s", input_topic_.c_str(),
      tracking_enabled_ ? "enabled" : "disabled");
  }

private:
  template<typename T>
  T parameterOr(
    const std::vector<rclcpp::Parameter> & updates, const std::string & name,
    T current) const
  {
    for (const auto & parameter : updates) {
      if (parameter.get_name() == name) {return parameter.get_value<T>();}
    }
    return current;
  }

  PipelineConfig readPipelineConfig()
  {
    PipelineConfig config;
    config.voxel_leaf_size = declare_parameter<double>("voxel.leaf_size", config.voxel_leaf_size);
    const auto roi_min = declare_parameter<std::vector<double>>(
      "roi.min", {config.roi_min.x(), config.roi_min.y(), config.roi_min.z()});
    const auto roi_max = declare_parameter<std::vector<double>>(
      "roi.max", {config.roi_max.x(), config.roi_max.y(), config.roi_max.z()});
    if (roi_min.size() != 3U || roi_max.size() != 3U) {
      throw std::invalid_argument("roi.min and roi.max must contain exactly 3 values");
    }
    config.roi_min = Eigen::Vector4f{static_cast<float>(roi_min[0]), static_cast<float>(roi_min[1]),
      static_cast<float>(roi_min[2]), 1.0F};
    config.roi_max = Eigen::Vector4f{static_cast<float>(roi_max[0]), static_cast<float>(roi_max[1]),
      static_cast<float>(roi_max[2]), 1.0F};
    config.outlier_method = declare_parameter<std::string>("outlier.method", config.outlier_method);
    config.statistical_mean_k = declare_parameter<int>("outlier.statistical_mean_k",
        config.statistical_mean_k);
    config.statistical_stddev = declare_parameter<double>("outlier.statistical_stddev",
        config.statistical_stddev);
    config.radius_search = declare_parameter<double>("outlier.radius_search", config.radius_search);
    config.radius_min_neighbors = declare_parameter<int>("outlier.radius_min_neighbors",
        config.radius_min_neighbors);
    config.ground_distance_threshold = declare_parameter<double>(
      "ground.distance_threshold", config.ground_distance_threshold);
    config.ground_max_iterations = declare_parameter<int>("ground.max_iterations",
        config.ground_max_iterations);
    config.ground_max_angle_deg = declare_parameter<double>("ground.max_angle_deg",
        config.ground_max_angle_deg);
    config.cluster_tolerance = declare_parameter<double>("cluster.tolerance",
        config.cluster_tolerance);
    config.cluster_min_size = declare_parameter<int>("cluster.min_size", config.cluster_min_size);
    config.cluster_max_size = declare_parameter<int>("cluster.max_size", config.cluster_max_size);
    config.oriented_boxes = declare_parameter<bool>("bbox.oriented", config.oriented_boxes);
    return config;
  }

  TrackerConfig readTrackerConfig()
  {
    TrackerConfig config;
    config.association_max_distance = declare_parameter<double>(
      "tracking.association_max_distance", config.association_max_distance);
    config.process_noise = declare_parameter<double>("tracking.process_noise",
        config.process_noise);
    config.measurement_noise = declare_parameter<double>("tracking.measurement_noise",
        config.measurement_noise);
    config.min_hits = declare_parameter<int>("tracking.min_hits", config.min_hits);
    config.max_misses = declare_parameter<int>("tracking.max_misses", config.max_misses);
    return config;
  }

  PipelineConfig updatedPipelineConfig(const std::vector<rclcpp::Parameter> & updates) const
  {
    auto c = processor_->config();
    c.voxel_leaf_size = static_cast<float>(parameterOr<double>(updates, "voxel.leaf_size",
        c.voxel_leaf_size));
    c.outlier_method = parameterOr<std::string>(updates, "outlier.method", c.outlier_method);
    c.statistical_mean_k = parameterOr<int>(updates, "outlier.statistical_mean_k",
        c.statistical_mean_k);
    c.statistical_stddev = parameterOr<double>(updates, "outlier.statistical_stddev",
        c.statistical_stddev);
    c.radius_search = parameterOr<double>(updates, "outlier.radius_search", c.radius_search);
    c.radius_min_neighbors = parameterOr<int>(updates, "outlier.radius_min_neighbors",
        c.radius_min_neighbors);
    c.ground_distance_threshold = parameterOr<double>(updates, "ground.distance_threshold",
        c.ground_distance_threshold);
    c.ground_max_iterations = parameterOr<int>(updates, "ground.max_iterations",
        c.ground_max_iterations);
    c.ground_max_angle_deg = parameterOr<double>(updates, "ground.max_angle_deg",
        c.ground_max_angle_deg);
    c.cluster_tolerance = parameterOr<double>(updates, "cluster.tolerance", c.cluster_tolerance);
    c.cluster_min_size = parameterOr<int>(updates, "cluster.min_size", c.cluster_min_size);
    c.cluster_max_size = parameterOr<int>(updates, "cluster.max_size", c.cluster_max_size);
    c.oriented_boxes = parameterOr<bool>(updates, "bbox.oriented", c.oriented_boxes);
    for (const auto & p : updates) {
      if (p.get_name() == "roi.min" || p.get_name() == "roi.max") {
        const auto values = p.as_double_array();
        if (values.size() != 3U) {
          throw std::invalid_argument(p.get_name() + " must contain exactly 3 values");
        }
        auto & target = p.get_name() == "roi.min" ? c.roi_min : c.roi_max;
        target = Eigen::Vector4f{static_cast<float>(values[0]), static_cast<float>(values[1]),
          static_cast<float>(values[2]), 1.0F};
      }
    }
    return c;
  }

  TrackerConfig updatedTrackerConfig(const std::vector<rclcpp::Parameter> & updates) const
  {
    TrackerConfig c;
    c.association_max_distance = get_parameter("tracking.association_max_distance").as_double();
    c.process_noise = get_parameter("tracking.process_noise").as_double();
    c.measurement_noise = get_parameter("tracking.measurement_noise").as_double();
    c.min_hits = static_cast<int>(get_parameter("tracking.min_hits").as_int());
    c.max_misses = static_cast<int>(get_parameter("tracking.max_misses").as_int());
    c.association_max_distance = parameterOr<double>(updates, "tracking.association_max_distance",
        c.association_max_distance);
    c.process_noise = parameterOr<double>(updates, "tracking.process_noise", c.process_noise);
    c.measurement_noise = parameterOr<double>(updates, "tracking.measurement_noise",
        c.measurement_noise);
    c.min_hits = parameterOr<int>(updates, "tracking.min_hits", c.min_hits);
    c.max_misses = parameterOr<int>(updates, "tracking.max_misses", c.max_misses);
    return c;
  }

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & updates)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    for (const auto & parameter : updates) {
      if (parameter.get_name() == "input_topic" || parameter.get_name() == "queue_depth") {
        result.reason = parameter.get_name() + " cannot be changed after startup";
        return result;
      }
    }
    try {
      auto pipeline = updatedPipelineConfig(updates);
      auto tracker_config = updatedTrackerConfig(updates);
      std::string reason;
      if (!Processor::validate(pipeline, reason) || !MultiObjectTracker::validate(tracker_config,
          reason))
      {
        result.reason = reason;
        return result;
      }
      processor_ = std::make_unique<Processor>(pipeline);
      tracker_ = std::make_unique<MultiObjectTracker>(tracker_config);
      tracking_enabled_ = parameterOr<bool>(updates, "tracking.enabled", tracking_enabled_);
      publish_debug_clouds_ = parameterOr<bool>(updates, "publish_debug_clouds",
          publish_debug_clouds_);
      result.successful = true;
      result.reason = "configuration applied; tracker state reset";
    } catch (const std::exception & error) {
      result.reason = error.what();
    }
    return result;
  }

  void publishCloud(
    const Cloud & cloud, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher->publish(message);
  }

  static geometry_msgs::msg::Pose toPose(const Detection & detection)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = detection.centroid.x();
    pose.position.y = detection.centroid.y();
    pose.position.z = detection.centroid.z();
    pose.orientation.x = detection.orientation.x();
    pose.orientation.y = detection.orientation.y();
    pose.orientation.z = detection.orientation.z();
    pose.orientation.w = detection.orientation.w();
    return pose;
  }

  void publishObjects(
    const std_msgs::msg::Header & header, const std::vector<Detection> & objects)
  {
    lidar_perception_msgs::msg::ClusterArray cluster_array;
    vision_msgs::msg::Detection3DArray detection_array;
    visualization_msgs::msg::MarkerArray markers;
    cluster_array.header = header;
    detection_array.header = header;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const auto & object = objects[i];
      const std::uint32_t object_id = object.tracked ? object.id : static_cast<std::uint32_t>(i);
      lidar_perception_msgs::msg::Cluster cluster;
      cluster.id = object_id;
      cluster.pose = toPose(object);
      cluster.size.x = object.size.x();
      cluster.size.y = object.size.y();
      cluster.size.z = object.size.z();
      cluster.velocity.x = object.velocity.x();
      cluster.velocity.y = object.velocity.y();
      cluster.velocity.z = object.velocity.z();
      cluster.point_count = static_cast<std::uint32_t>(object.point_count);
      cluster.tracked = object.tracked;
      cluster_array.clusters.push_back(cluster);

      vision_msgs::msg::Detection3D detection;
      detection.header = header;
      detection.id = std::to_string(object_id);
      detection.bbox.center = cluster.pose;
      detection.bbox.size = cluster.size;
      detection_array.detections.push_back(detection);

      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "boxes";
      box.id = marker_id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose = cluster.pose;
      box.scale = cluster.size;
      box.color.r = object.tracked ? 0.1F : 1.0F;
      box.color.g = object.tracked ? 0.9F : 0.6F;
      box.color.b = 0.2F;
      box.color.a = 0.25F;
      box.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(box);

      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = cluster.pose;
      label.pose.position.z += 0.5 * object.size.z() + 0.3;
      label.scale.z = 0.35;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
      label.text = (object.tracked ? "track " : "cluster ") + std::to_string(object_id);
      if (object.tracked) {
        label.text += "  " + std::to_string(std::hypot(object.velocity.x(),
            object.velocity.y())).substr(0, 4) + " m/s";
      }
      label.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(label);
    }
    cluster_publisher_->publish(cluster_array);
    detection_publisher_->publish(detection_array);
    marker_publisher_->publish(markers);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const auto start = std::chrono::steady_clock::now();
    try {
      auto cloud = std::make_shared<Cloud>();
      pcl::fromROSMsg(*message, *cloud);
      auto result = processor_->process(cloud);
      const double stamp = rclcpp::Time(message->header.stamp).seconds();
      auto objects = tracking_enabled_ ? tracker_->update(result.detections,
          stamp) : result.detections;

      if (publish_debug_clouds_) {
        publishCloud(*result.filtered, message->header, filtered_publisher_);
        publishCloud(*result.ground, message->header, ground_publisher_);
        publishCloud(*result.nonground, message->header, nonground_publisher_);
      }
      publishObjects(message->header, objects);

      const auto end = std::chrono::steady_clock::now();
      last_latency_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
      if (last_end_time_.time_since_epoch().count() != 0) {
        const double instantaneous_hz = 1.0 /
          std::chrono::duration<double>(end - last_end_time_).count();
        processing_hz_ = processing_hz_ ==
          0.0 ? instantaneous_hz : 0.9 * processing_hz_ + 0.1 * instantaneous_hz;
      }
      last_end_time_ = end;
      last_input_points_ = result.input_points;
      last_filtered_points_ = result.filtered->size();
      last_ground_points_ = result.ground->size();
      last_nonground_points_ = result.nonground->size();
      last_cluster_count_ = result.detections.size();
      last_track_count_ = objects.size();
      last_error_.clear();
      frames_processed_++;

      lidar_perception_msgs::msg::PipelineMetrics metrics;
      metrics.header = message->header;
      metrics.input_points = last_input_points_;
      metrics.filtered_points = last_filtered_points_;
      metrics.ground_points = last_ground_points_;
      metrics.nonground_points = last_nonground_points_;
      metrics.cluster_count = last_cluster_count_;
      metrics.track_count = last_track_count_;
      metrics.latency_ms = last_latency_ms_;
      metrics.processing_hz = processing_hz_;
      metrics_publisher_->publish(metrics);
      updater_.force_update();
    } catch (const std::exception & error) {
      last_error_ = error.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Frame processing failed: %s",
          error.what());
      updater_.force_update();
    }
  }

  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (!last_error_.empty()) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, last_error_);
    } else if (frames_processed_ == 0U) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for point clouds");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Pipeline operational");
    }
    status.add("frames_processed", frames_processed_);
    status.add("latency_ms", last_latency_ms_);
    status.add("processing_hz", processing_hz_);
    status.add("input_points", last_input_points_);
    status.add("filtered_points", last_filtered_points_);
    status.add("ground_points", last_ground_points_);
    status.add("nonground_points", last_nonground_points_);
    status.add("clusters", last_cluster_count_);
    status.add("published_tracks", last_track_count_);
    status.add("tracking_enabled", tracking_enabled_);
  }

  std::string input_topic_;
  int queue_depth_{5};
  bool tracking_enabled_{true};
  bool publish_debug_clouds_{true};
  std::unique_ptr<Processor> processor_;
  std::unique_ptr<MultiObjectTracker> tracker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;
  rclcpp::Publisher<lidar_perception_msgs::msg::ClusterArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<lidar_perception_msgs::msg::PipelineMetrics>::SharedPtr metrics_publisher_;
  diagnostic_updater::Updater updater_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::chrono::steady_clock::time_point last_end_time_{};
  double last_latency_ms_{0.0};
  double processing_hz_{0.0};
  std::size_t last_input_points_{0};
  std::size_t last_filtered_points_{0};
  std::size_t last_ground_points_{0};
  std::size_t last_nonground_points_{0};
  std::size_t last_cluster_count_{0};
  std::size_t last_track_count_{0};
  std::size_t frames_processed_{0};
  std::string last_error_;
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
