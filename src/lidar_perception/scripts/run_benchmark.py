#!/usr/bin/env python3

# Copyright 2026 ros2_lidar_perception contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Launch the deterministic workload and record observed pipeline metrics."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import sys
import time

from lidar_perception_msgs.msg import ClusterArray, PipelineMetrics
import rclpy
from rclpy.node import Node


class Collector(Node):
    """Collect metric and tracking samples emitted by the pipeline."""

    def __init__(self, warmup_seconds):
        super().__init__('lidar_perception_benchmark')
        self.start_time = time.monotonic()
        self.warmup_seconds = warmup_seconds
        self.metrics = []
        self.track_ids = []
        self.create_subscription(
            PipelineMetrics,
            '/lidar_perception/metrics',
            self._metrics_callback,
            50,
        )
        self.create_subscription(
            ClusterArray,
            '/lidar_perception/clusters',
            self._clusters_callback,
            50,
        )

    def warmed_up(self):
        """Return true after the configured warm-up interval."""
        return time.monotonic() - self.start_time >= self.warmup_seconds

    def _metrics_callback(self, message):
        if self.warmed_up():
            self.metrics.append(message)

    def _clusters_callback(self, message):
        if self.warmed_up():
            self.track_ids.append(sorted(c.id for c in message.clusters if c.tracked))


def distribution(values):
    """Return basic descriptive statistics without third-party modules."""
    ordered = sorted(values)
    if not ordered:
        return None

    def percentile(fraction):
        index = min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))
        return ordered[index]

    return {
        'mean': statistics.fmean(values),
        'median': statistics.median(values),
        'p95': percentile(0.95),
        'min': ordered[0],
        'max': ordered[-1],
    }


def cpu_model():
    """Read a best-effort CPU model without adding a platform dependency."""
    try:
        for line in Path('/proc/cpuinfo').read_text(encoding='utf-8').splitlines():
            if line.startswith('model name'):
                return line.split(':', maxsplit=1)[1].strip()
    except (OSError, IndexError):
        pass
    return platform.processor() or 'unknown'


def summarize(collector, duration, build_type, rmw_implementation):
    """Build the machine-readable benchmark report."""
    samples = collector.metrics
    latencies = [m.latency_ms for m in samples]
    frequencies = [m.processing_hz for m in samples if m.processing_hz > 0.0]
    reductions = [
        1.0 - m.filtered_points / m.input_points
        for m in samples
        if m.input_points > 0
    ]
    cluster_counts = [m.cluster_count for m in samples]
    track_counts = [m.track_count for m in samples]
    transitions = 0
    retained = 0
    for previous, current in zip(collector.track_ids, collector.track_ids[1:]):
        previous_set = set(previous)
        current_set = set(current)
        transitions += len(previous_set)
        retained += len(previous_set & current_set)
    return {
        'provenance': {
            'workload': 'deterministic synthetic.launch.py',
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'platform': platform.platform(),
            'cpu_model': cpu_model(),
            'logical_cpu_count': os.cpu_count(),
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
            'rmw_implementation': rmw_implementation,
            'build_type': build_type,
            'duration_seconds': duration,
            'sample_count': len(samples),
            'clock': 'steady_clock for latency; ROS messages for counts',
        },
        'latency_ms': distribution(latencies),
        'processing_hz': distribution(frequencies),
        'point_reduction_ratio': distribution(reductions),
        'cluster_count': distribution(cluster_counts),
        'published_track_count': distribution(track_counts),
        'tracking': {
            'observed_frames': len(collector.track_ids),
            'id_retention_ratio': retained / transitions if transitions else None,
            'note': 'Ratio of prior-frame confirmed IDs retained in the next frame.',
        },
    }


def parse_args():
    """Parse command-line options."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--warmup', type=float, default=3.0)
    parser.add_argument('--output', type=Path, default=Path('results/benchmark.json'))
    parser.add_argument('--build-type', default='unspecified')
    parser.add_argument('--no-launch', action='store_true')
    return parser.parse_args()


def main():
    """Run the benchmark and write only measurements observed in this run."""
    args = parse_args()
    if args.duration <= 0.0 or args.warmup < 0.0:
        raise ValueError('duration must be positive and warmup non-negative')
    process = None
    if not args.no_launch:
        process = subprocess.Popen(
            ['ros2', 'launch', 'lidar_perception', 'synthetic.launch.py'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    rclpy.init()
    rmw_implementation = rclpy.get_rmw_implementation_identifier()
    collector = Collector(args.warmup)
    deadline = time.monotonic() + args.warmup + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.2)
            if process and process.poll() is not None:
                raise RuntimeError('synthetic launch exited before benchmark completed')
    finally:
        collector.destroy_node()
        rclpy.shutdown()
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
    report = summarize(collector, args.duration, args.build_type, rmw_implementation)
    if report['provenance']['sample_count'] == 0:
        raise RuntimeError('no metrics received; is the workspace sourced?')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(f'Wrote {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
