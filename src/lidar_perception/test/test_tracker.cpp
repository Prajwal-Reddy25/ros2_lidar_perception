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

#include <set>
#include <string>
#include <vector>

#include "lidar_perception/tracker.hpp"

namespace lidar_perception
{

namespace
{
Detection detectionAt(const float x, const float y)
{
  Detection detection;
  detection.centroid = Eigen::Vector3f{x, y, 1.0F};
  detection.size = Eigen::Vector3f{4.0F, 2.0F, 1.5F};
  detection.point_count = 100;
  return detection;
}
}  // namespace

TEST(Tracker, MaintainsIdsAndEstimatesVelocity)
{
  TrackerConfig config;
  config.min_hits = 2;
  config.association_max_distance = 2.0;
  MultiObjectTracker tracker(config);
  EXPECT_TRUE(tracker.update({detectionAt(0.0F, 0.0F), detectionAt(10.0F, 2.0F)}, 1.0).empty());
  const auto second = tracker.update({detectionAt(0.5F, 0.0F), detectionAt(10.0F, 2.0F)}, 1.1);
  ASSERT_EQ(second.size(), 2U);
  const std::set<std::uint32_t> ids{second[0].id, second[1].id};
  EXPECT_EQ(ids.size(), 2U);
  const auto third = tracker.update({detectionAt(1.0F, 0.0F), detectionAt(10.0F, 2.0F)}, 1.2);
  ASSERT_EQ(third.size(), 2U);
  EXPECT_EQ(third[0].id, second[0].id);
  EXPECT_GT(third[0].velocity.x(), 0.0F);
  EXPECT_TRUE(third[0].tracked);
}

TEST(Tracker, RemovesStaleTracks)
{
  TrackerConfig config;
  config.min_hits = 1;
  config.max_misses = 1;
  MultiObjectTracker tracker(config);
  EXPECT_EQ(tracker.update({detectionAt(1.0F, 1.0F)}, 1.0).size(), 1U);
  EXPECT_EQ(tracker.update({}, 1.1).size(), 1U);
  EXPECT_TRUE(tracker.update({}, 1.2).empty());
  EXPECT_EQ(tracker.trackCount(), 0U);
}

TEST(Tracker, RejectsInvalidConfiguration)
{
  TrackerConfig config;
  config.association_max_distance = -1.0;
  std::string reason;
  EXPECT_FALSE(MultiObjectTracker::validate(config, reason));
  EXPECT_THROW(MultiObjectTracker tracker(config), std::invalid_argument);
}

}  // namespace lidar_perception
