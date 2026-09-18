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


#include "lidar_perception/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace lidar_perception
{

MultiObjectTracker::MultiObjectTracker(TrackerConfig config)
: config_(std::move(config))
{
  std::string reason;
  if (!validate(config_, reason)) {throw std::invalid_argument(reason);}
}

bool MultiObjectTracker::validate(const TrackerConfig & c, std::string & reason)
{
  if (c.association_max_distance <= 0.0) {
    reason = "association_max_distance must be positive"; return false;
  }
  if (c.process_noise <= 0.0 || c.measurement_noise <= 0.0) {
    reason = "tracker noise values must be positive"; return false;
  }
  if (c.min_hits < 1 || c.max_misses < 0) {
    reason = "invalid tracker lifecycle parameters"; return false;
  }
  return true;
}

void MultiObjectTracker::reset()
{
  tracks_.clear();
  next_id_ = 1;
  has_stamp_ = false;
}

void MultiObjectTracker::predict(Track & track, const double dt) const
{
  Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
  transition(0, 2) = dt;
  transition(1, 3) = dt;
  const double q = config_.process_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
  noise << dt4 / 4.0, 0.0, dt3 / 2.0, 0.0,
    0.0, dt4 / 4.0, 0.0, dt3 / 2.0,
    dt3 / 2.0, 0.0, dt2, 0.0,
    0.0, dt3 / 2.0, 0.0, dt2;
  track.state = transition * track.state;
  track.covariance = transition * track.covariance * transition.transpose() + q * noise;
}

void MultiObjectTracker::correct(Track & track, const Detection & detection) const
{
  Eigen::Matrix<double, 2, 4> observation = Eigen::Matrix<double, 2, 4>::Zero();
  observation(0, 0) = 1.0;
  observation(1, 1) = 1.0;
  const Eigen::Matrix2d noise = config_.measurement_noise * Eigen::Matrix2d::Identity();
  const Eigen::Vector2d measurement{detection.centroid.x(), detection.centroid.y()};
  const Eigen::Vector2d innovation = measurement - observation * track.state;
  const Eigen::Matrix2d innovation_covariance =
    observation * track.covariance * observation.transpose() + noise;
  const Eigen::Matrix<double, 4, 2> gain =
    track.covariance * observation.transpose() * innovation_covariance.inverse();
  track.state += gain * innovation;
  track.covariance =
    (Eigen::Matrix4d::Identity() - gain * observation) * track.covariance;
  track.shape = detection;
  track.hits++;
  track.misses = 0;
}

std::vector<Detection> MultiObjectTracker::update(
  const std::vector<Detection> & detections, const double stamp_seconds)
{
  double dt = 0.1;
  if (has_stamp_) {
    dt = std::clamp(stamp_seconds - last_stamp_seconds_, 0.001, 1.0);
  }
  last_stamp_seconds_ = stamp_seconds;
  has_stamp_ = true;
  for (auto & track : tracks_) {
    predict(track, dt); track.misses++;
  }

  using Candidate = std::tuple<double, std::size_t, std::size_t>;
  std::vector<Candidate> candidates;
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    for (std::size_t di = 0; di < detections.size(); ++di) {
      const double dx = tracks_[ti].state.x() - detections[di].centroid.x();
      const double dy = tracks_[ti].state.y() - detections[di].centroid.y();
      const double distance = std::hypot(dx, dy);
      if (distance <= config_.association_max_distance) {
        candidates.emplace_back(distance, ti, di);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());
  std::vector<bool> used_tracks(tracks_.size(), false);
  std::vector<bool> used_detections(detections.size(), false);
  for (const auto & [distance, ti, di] : candidates) {
    (void)distance;
    if (!used_tracks[ti] && !used_detections[di]) {
      correct(tracks_[ti], detections[di]);
      used_tracks[ti] = true;
      used_detections[di] = true;
    }
  }

  for (std::size_t di = 0; di < detections.size(); ++di) {
    if (used_detections[di]) {continue;}
    Track track;
    track.id = next_id_++;
    track.state << detections[di].centroid.x(), detections[di].centroid.y(), 0.0, 0.0;
    track.covariance = Eigen::Matrix4d::Identity();
    track.shape = detections[di];
    tracks_.push_back(track);
  }
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [this](const Track & track) {
      return track.misses > config_.max_misses;
    }), tracks_.end());

  std::vector<Detection> output;
  for (const auto & track : tracks_) {
    if (track.hits < config_.min_hits) {continue;}
    Detection detection = track.shape;
    detection.id = track.id;
    detection.centroid.x() = static_cast<float>(track.state.x());
    detection.centroid.y() = static_cast<float>(track.state.y());
    detection.velocity.x() = static_cast<float>(track.state.z());
    detection.velocity.y() = static_cast<float>(track.state.w());
    detection.tracked = true;
    output.push_back(detection);
  }
  return output;
}

}  // namespace lidar_perception
