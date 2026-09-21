# ros2_lidar_perception

[![ROS 2 Jazzy CI](https://github.com/Prajwal-Reddy25/ros2_lidar_perception/actions/workflows/ci.yml/badge.svg)](https://github.com/Prajwal-Reddy25/ros2_lidar_perception/actions/workflows/ci.yml)

A production-oriented, hardware-independent 3D LiDAR obstacle pipeline for
Ubuntu 24.04 and ROS 2 Jazzy. It consumes `sensor_msgs/PointCloud2`, produces
ground and nonground point clouds, clustered 3D boxes and box centers, optional
temporally consistent track IDs and estimated velocities, RViz markers,
standard diagnostics, and typed metrics.

The hot path is C++17/PCL. A deterministic synthetic scene and PCD player make
the complete graph usable without LiDAR hardware.

## Features

- finite-point cleanup, voxel downsampling, and configurable XYZ ROI
- optional statistical or radius outlier filtering
- Z-constrained RANSAC ground plane and nonground extraction
- k-d-tree Euclidean clustering with size gates
- axis-aligned or yaw-oriented PCA bounding boxes
- box center, dimensions, point count, and `vision_msgs/Detection3DArray`
- optional constant-velocity Kalman tracking with gated data association
- debug clouds, RViz markers, standard `/diagnostics`, and frame metrics
- deterministic synthetic publisher and ordered PCD playback
- GTest coverage, lint integration, GitHub Actions, and JSON benchmark tooling

## Build

Install ROS 2 Jazzy Desktop (or ROS Base plus RViz), PCL, and the package
dependencies. From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Run

### Synthetic Demo

![Synthetic 3D LiDAR perception pipeline in RViz](docs/evidence/rviz_synthetic.png)

The deterministic synthetic scene exercises ground segmentation, Euclidean clustering,
3D bounding-box estimation, and optional multi-object tracking without requiring
physical LiDAR hardware.

```bash
ros2 launch lidar_perception synthetic.launch.py rviz:=true
```

The fixed frame is `lidar`. The synthetic scene contains a noisy planar road,
three cuboids, sparse outliers, and one moving cuboid. Generation is seeded, so
regression runs are repeatable while each scan still has different samples.

### Real Sensor or Rosbag

For a real sensor or rosbag that publishes `/points_raw`:

```bash
ros2 launch lidar_perception pipeline.launch.py
```

Override or remap the point-cloud topic as required.

### PCD Playback

For PCD data:

```bash
ros2 launch lidar_perception dataset.launch.py \
	path:=/data/pcd_sequence \
	rate:=10
```

The PCD player accepts one `.pcd` file or a directory and processes files in  
lexical order.

## Configure

All algorithm settings are in
[`perception.yaml`](src/lidar_perception/config/perception.yaml). Parameters can
be changed at runtime except for the subscription topic and queue depth, which
are startup-only. Invalid updates are rejected as a set, and accepted updates
reset tracker state. Important choices are:

- `outlier.method`: `none`, `statistical`, or `radius`
- `bbox.oriented`: PCA yaw box when true, AABB when false
- `tracking.enabled`: bypass or enable confirmed-track output
- `publish_debug_clouds`: disable three large cloud publications in deployment

Namespaced output topics resolve beneath `/lidar_perception`. See
[`ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the exact graph and contracts and
[`ALGORITHMS.md`](docs/ALGORITHMS.md) for assumptions and tuning guidance.

## Test and benchmark

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

ros2 run lidar_perception run_benchmark.py \
  --warmup 3 --duration 30 --output results/benchmark.json
```

The benchmark prints and writes only values observed from `PipelineMetrics` in
that run. No performance claim is hard-coded. Interpretation and limitations
are documented in [`BENCHMARKING.md`](docs/BENCHMARKING.md).

## Validation Snapshot

The checked-in synthetic benchmark was recorded using ROS 2 Jazzy, Fast DDS,
a `RelWithDebInfo` build, and the documented seeded synthetic workload.

| Metric | Result |
| --- | ---: |
| Samples | 102 |
| Mean callback latency | 13.178 ms |
| p95 callback latency | 14.107 ms |
| Observed workload rate | 10.003 Hz |
| Clusters per sampled frame | 3.0 |
| Confirmed tracks per sampled frame | 3.0 |
| Adjacent-frame ID retention | 1.0 |
| Test-result entries | 67 |
| Errors / failures | 0 / 0 |
| Skipped | 10 (`cppcheck` safeguard in ROS 2 Jazzy) |
| CTest | 10/10 passed |
| GitHub Actions | Passed |

The 10.003 Hz value reflects the configured synthetic input rate and is **not**
a maximum-throughput measurement. Results apply only to the documented host,
middleware, parameters, build, and synthetic workload.

See [`docs/evidence/README.md`](docs/evidence/README.md) and
[`docs/evidence/synthetic_benchmark.json`](docs/evidence/synthetic_benchmark.json)
for the full evidence and provenance.

## Evidence

The checked-in [validation evidence](docs/evidence/README.md) includes an actual
RViz capture and the raw JSON from a reproducible synthetic benchmark run.

## Repository Layout

```text
src/
├── lidar_perception_msgs/   ROS 2 message contracts
└── lidar_perception/        C++ perception library, nodes, launch/config/RViz, tests

docs/                        Architecture, algorithms, benchmarking, evidence
.github/workflows/            ROS 2 Jazzy CI
```

## Operational notes

- The incoming frame is preserved; no implicit TF transform is performed.
- The baseline uses one dominant ground plane and fixed-radius clustering.
  Sloped/undulating roads and strongly range-dependent density need more
  advanced segmentation/tolerance models.
- Synthetic geometry validates behavior, integration, and repeatability. It is
  not a substitute for labeled-dataset precision/recall evaluation.
- Track IDs are local to a process lifetime and parameter configuration.

Licensed under the [MIT License](LICENSE).
