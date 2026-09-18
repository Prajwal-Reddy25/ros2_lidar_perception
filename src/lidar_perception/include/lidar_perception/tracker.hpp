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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "lidar_perception/types.hpp"

namespace lidar_perception
{

struct TrackerConfig
{
  double association_max_distance{2.5};
  double process_noise{2.0};
  double measurement_noise{0.25};
  int min_hits{2};
  int max_misses{4};
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(TrackerConfig config = {});

  std::vector<Detection> update(
    const std::vector<Detection> & detections, double stamp_seconds);
  void reset();
  [[nodiscard]] std::size_t trackCount() const noexcept {return tracks_.size();}
  static bool validate(const TrackerConfig & config, std::string & reason);

private:
  struct Track
  {
    std::uint32_t id{0};
    Eigen::Vector4d state{Eigen::Vector4d::Zero()};
    Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};
    Detection shape;
    int hits{1};
    int misses{0};
  };

  void predict(Track & track, double dt) const;
  void correct(Track & track, const Detection & detection) const;

  TrackerConfig config_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
  double last_stamp_seconds_{0.0};
  bool has_stamp_{false};
};

}  // namespace lidar_perception
