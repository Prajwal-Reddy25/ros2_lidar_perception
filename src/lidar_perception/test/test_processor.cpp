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


#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "lidar_perception/processor.hpp"

namespace lidar_perception
{

TEST(Processor, RejectsInvalidConfiguration)
{
  PipelineConfig config;
  config.voxel_leaf_size = 0.0F;
  std::string reason;
  EXPECT_FALSE(Processor::validate(config, reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_THROW(Processor processor(config), std::invalid_argument);
}

TEST(Processor, SegmentsGroundAndClustersObstacles)
{
  PipelineConfig config;
  config.voxel_leaf_size = 0.08F;
  config.roi_min = Eigen::Vector4f{-10.0F, -10.0F, -1.0F, 1.0F};
  config.roi_max = Eigen::Vector4f{20.0F, 10.0F, 4.0F, 1.0F};
  config.ground_distance_threshold = 0.08;
  config.cluster_tolerance = 0.45;
  config.cluster_min_size = 10;
  config.oriented_boxes = false;
  Processor processor(config);
  auto cloud = std::make_shared<Cloud>();
  for (int x = -50; x <= 100; ++x) {
    for (int y = -40; y <= 40; y += 2) {
      cloud->push_back(Point{0.1F * x, 0.1F * y, 0.0F, 1.0F});
    }
  }
  for (const auto & center : {Eigen::Vector2f{3.0F, 2.0F}, Eigen::Vector2f{8.0F, -2.0F}}) {
    for (int x = -5; x <= 5; ++x) {
      for (int y = -4; y <= 4; ++y) {
        for (int z = 1; z <= 8; ++z) {
          cloud->push_back(Point{
              center.x() + 0.08F * x, center.y() + 0.08F * y, 0.12F * z, 80.0F});
        }
      }
    }
  }
  cloud->push_back(Point{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F});
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  const auto result = processor.process(cloud);
  EXPECT_EQ(result.input_points, cloud->size());
  EXPECT_GT(result.ground->size(), 1000U);
  EXPECT_GT(result.nonground->size(), 100U);
  ASSERT_EQ(result.detections.size(), 2U);
  EXPECT_NEAR(result.detections[0].size.z(), 0.84F, 0.15F);
  EXPECT_GT(result.detections[0].point_count, 50U);
}

TEST(Processor, RoiRemovesPointsOutsideConfiguredBounds)
{
  PipelineConfig config;
  config.roi_min = Eigen::Vector4f{-1.0F, -1.0F, -1.0F, 1.0F};
  config.roi_max = Eigen::Vector4f{1.0F, 1.0F, 1.0F, 1.0F};
  config.cluster_min_size = 1;
  Processor processor(config);
  auto cloud = std::make_shared<Cloud>();
  cloud->push_back(Point{0.0F, 0.0F, 0.0F, 1.0F});
  cloud->push_back(Point{20.0F, 0.0F, 0.0F, 1.0F});
  const auto result = processor.process(cloud);
  EXPECT_EQ(result.filtered->size(), 1U);
}

}  // namespace lidar_perception
