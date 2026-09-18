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


#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lidar_perception
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct PipelineConfig
{
  float voxel_leaf_size{0.15F};
  Eigen::Vector4f roi_min{-30.0F, -15.0F, -3.0F, 1.0F};
  Eigen::Vector4f roi_max{60.0F, 15.0F, 5.0F, 1.0F};
  std::string outlier_method{"none"};
  int statistical_mean_k{20};
  double statistical_stddev{1.0};
  double radius_search{0.4};
  int radius_min_neighbors{2};
  double ground_distance_threshold{0.18};
  int ground_max_iterations{100};
  double ground_max_angle_deg{15.0};
  double cluster_tolerance{0.55};
  int cluster_min_size{8};
  int cluster_max_size{20000};
  bool oriented_boxes{true};
};

struct Detection
{
  std::uint32_t id{0};
  Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
  Eigen::Vector3f size{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
  std::size_t point_count{0};
  bool tracked{false};
};

struct ProcessingResult
{
  Cloud::Ptr filtered{new Cloud};
  Cloud::Ptr ground{new Cloud};
  Cloud::Ptr nonground{new Cloud};
  std::vector<Detection> detections;
  std::size_t input_points{0};
};

}  // namespace lidar_perception
