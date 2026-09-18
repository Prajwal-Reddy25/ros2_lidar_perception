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


#include "lidar_perception/processor.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_perception
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Processor::Processor(PipelineConfig config)
: config_(std::move(config))
{
  std::string reason;
  if (!validate(config_, reason)) {
    throw std::invalid_argument(reason);
  }
}

bool Processor::validate(const PipelineConfig & c, std::string & reason)
{
  if (c.voxel_leaf_size <= 0.0F) {reason = "voxel_leaf_size must be positive"; return false;}
  if ((c.roi_min.head<3>().array() >= c.roi_max.head<3>().array()).any()) {
    reason = "each roi_min must be less than roi_max";
    return false;
  }
  if (c.outlier_method != "none" && c.outlier_method != "statistical" &&
    c.outlier_method != "radius")
  {
    reason = "outlier_method must be none, statistical, or radius"; return false;
  }
  if (c.statistical_mean_k < 2 || c.statistical_stddev <= 0.0) {
    reason = "invalid statistical outlier parameters"; return false;
  }
  if (c.radius_search <= 0.0 || c.radius_min_neighbors < 1) {
    reason = "invalid radius outlier parameters"; return false;
  }
  if (c.ground_distance_threshold <= 0.0 || c.ground_max_iterations < 1 ||
    c.ground_max_angle_deg <= 0.0 || c.ground_max_angle_deg >= 90.0)
  {
    reason = "invalid ground segmentation parameters"; return false;
  }
  if (c.cluster_tolerance <= 0.0 || c.cluster_min_size < 1 ||
    c.cluster_max_size < c.cluster_min_size)
  {
    reason = "invalid clustering parameters"; return false;
  }
  return true;
}

ProcessingResult Processor::process(const Cloud::ConstPtr & input) const
{
  ProcessingResult result;
  if (!input) {return result;}
  result.input_points = input->size();

  auto finite = std::make_shared<Cloud>();
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*input, *finite, valid_indices);

  auto voxelized = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(finite);
  voxel.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
  voxel.filter(*voxelized);

  pcl::CropBox<Point> crop;
  crop.setInputCloud(voxelized);
  crop.setMin(config_.roi_min);
  crop.setMax(config_.roi_max);
  crop.filter(*result.filtered);

  if (config_.outlier_method == "statistical" && !result.filtered->empty()) {
    auto denoised = std::make_shared<Cloud>();
    pcl::StatisticalOutlierRemoval<Point> filter;
    filter.setInputCloud(result.filtered);
    filter.setMeanK(config_.statistical_mean_k);
    filter.setStddevMulThresh(config_.statistical_stddev);
    filter.filter(*denoised);
    result.filtered = denoised;
  } else if (config_.outlier_method == "radius" && !result.filtered->empty()) {
    auto denoised = std::make_shared<Cloud>();
    pcl::RadiusOutlierRemoval<Point> filter;
    filter.setInputCloud(result.filtered);
    filter.setRadiusSearch(config_.radius_search);
    filter.setMinNeighborsInRadius(config_.radius_min_neighbors);
    filter.filter(*denoised);
    result.filtered = denoised;
  }

  if (result.filtered->size() < 3U) {
    *result.nonground = *result.filtered;
    return result;
  }

  auto ground_indices = std::make_shared<pcl::PointIndices>();
  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  pcl::SACSegmentation<Point> segmenter;
  segmenter.setOptimizeCoefficients(true);
  segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmenter.setMethodType(pcl::SAC_RANSAC);
  segmenter.setAxis(Eigen::Vector3f::UnitZ());
  segmenter.setEpsAngle(config_.ground_max_angle_deg * kPi / 180.0);
  segmenter.setDistanceThreshold(config_.ground_distance_threshold);
  segmenter.setMaxIterations(config_.ground_max_iterations);
  segmenter.setInputCloud(result.filtered);
  segmenter.segment(*ground_indices, *coefficients);

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(result.filtered);
  extract.setIndices(ground_indices);
  extract.setNegative(false);
  extract.filter(*result.ground);
  extract.setNegative(true);
  extract.filter(*result.nonground);

  if (result.nonground->empty()) {return result;}
  auto tree = std::make_shared<pcl::search::KdTree<Point>>();
  tree->setInputCloud(result.nonground);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> clustering;
  clustering.setClusterTolerance(config_.cluster_tolerance);
  clustering.setMinClusterSize(config_.cluster_min_size);
  clustering.setMaxClusterSize(config_.cluster_max_size);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(result.nonground);
  clustering.extract(cluster_indices);

  result.detections.reserve(cluster_indices.size());
  for (const auto & indices : cluster_indices) {
    Cloud cluster;
    cluster.reserve(indices.indices.size());
    for (const int index : indices.indices) {
      cluster.push_back((*result.nonground)[index]);
    }
    result.detections.push_back(computeBox(cluster));
  }
  return result;
}

Detection Processor::computeBox(const Cloud & cloud) const
{
  Detection detection;
  detection.point_count = cloud.size();
  if (cloud.empty()) {return detection;}

  Point min_point;
  Point max_point;
  pcl::getMinMax3D(cloud, min_point, max_point);
  if (!config_.oriented_boxes || cloud.size() < 3U) {
    detection.centroid = 0.5F * Eigen::Vector3f{
      min_point.x + max_point.x, min_point.y + max_point.y, min_point.z + max_point.z};
    detection.size = Eigen::Vector3f{
      max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
    return detection;
  }

  Eigen::Vector4f mean;
  pcl::compute3DCentroid(cloud, mean);
  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (const auto & p : cloud) {
    const Eigen::Vector2f delta{p.x - mean.x(), p.y - mean.y()};
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= static_cast<float>(cloud.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
  Eigen::Vector2f axis = solver.eigenvectors().col(1);
  if (axis.x() < 0.0F) {axis = -axis;}
  const float yaw = std::atan2(axis.y(), axis.x());
  const Eigen::Rotation2Df inverse_rotation(-yaw);
  Eigen::Vector2f local_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f local_max = Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());
  for (const auto & p : cloud) {
    const Eigen::Vector2f local = inverse_rotation * Eigen::Vector2f{p.x, p.y};
    local_min = local_min.cwiseMin(local);
    local_max = local_max.cwiseMax(local);
  }
  const Eigen::Vector2f center_xy = Eigen::Rotation2Df(yaw) * (0.5F * (local_min + local_max));
  detection.centroid = Eigen::Vector3f{
    center_xy.x(), center_xy.y(), 0.5F * (min_point.z + max_point.z)};
  detection.size = Eigen::Vector3f{
    local_max.x() - local_min.x(), local_max.y() - local_min.y(), max_point.z - min_point.z};
  detection.orientation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  return detection;
}

}  // namespace lidar_perception
