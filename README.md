# ros2_lidar_perception

A production-oriented, hardware-independent 3D LiDAR obstacle pipeline for
Ubuntu 24.04 and ROS 2 Jazzy. It consumes `sensor_msgs/PointCloud2`, produces
ground/nonground clouds, clustered 3D boxes and centroids, optional stable track
IDs and velocities, RViz markers, standard diagnostics, and typed metrics.

The hot path is C++17/PCL. A deterministic synthetic scene and PCD player make
the complete graph usable without LiDAR hardware.

## Features

- finite-point cleanup, voxel downsampling, and configurable XYZ ROI
- optional statistical or radius outlier filtering
- Z-constrained RANSAC ground plane and nonground extraction
- k-d-tree Euclidean clustering with size gates
- axis-aligned or yaw-oriented PCA bounding boxes
- centroid, dimensions, point count, and `vision_msgs/Detection3DArray`
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

## Run without hardware

```bash
ros2 launch lidar_perception synthetic.launch.py rviz:=true
```

The fixed frame is `lidar`. The synthetic scene contains a noisy planar road,
three cuboids, sparse outliers, and one moving cuboid. Generation is seeded, so
regression runs are repeatable while each scan still has different samples.

To run a real sensor or rosbag that publishes `/points_raw`:

```bash
ros2 launch lidar_perception pipeline.launch.py
```

Override the topic in a copied YAML or at node invocation. For PCD data:

```bash
ros2 launch lidar_perception dataset.launch.py path:=/data/pcd_sequence rate:=10
```

The PCD player accepts one `.pcd` file or a directory, sorted lexically. For
rosbag2, use normal `ros2 bag play` and remap its cloud topic to `/points_raw`.

## Configure

All algorithm settings are in
[`perception.yaml`](src/lidar_perception/config/perception.yaml). Parameters can
be changed at runtime; invalid values are rejected as a set, and accepted
changes reset tracker state. Important choices are:

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

## Evidence

The checked-in [validation evidence](docs/evidence/README.md) includes an actual
RViz capture and the raw JSON from a reproducible synthetic benchmark run.

## Repository layout

```text
src/lidar_perception_msgs/  Stable ROS message contracts
src/lidar_perception/       C++ library, nodes, launch/config/RViz, tests
docs/                       Architecture, algorithms, benchmark evidence
.github/workflows/          Jazzy build/test CI
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
