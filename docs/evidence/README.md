# Validation evidence

This evidence was generated locally on 2026-09-18 from the repository build,
not estimated or copied from another system.

![RViz showing synthetic ground/nonground points, tracked clusters, and labels](rviz_synthetic.png)

The screenshot is the supplied RViz configuration connected to the seeded
synthetic publisher and perception node. Green points are RANSAC ground,
orange points are nonground, and translucent boxes/labels are confirmed tracks.

## Synthetic benchmark

Command:

```bash
ROS_DOMAIN_ID=89 ros2 run lidar_perception run_benchmark.py \
  --warmup 2 --duration 10 --build-type RelWithDebInfo \
  --output docs/evidence/synthetic_benchmark.json
```

Observed after warm-up on an AMD Ryzen 9 8940HX (32 logical CPUs), ROS 2 Jazzy,
Fast DDS, and a `RelWithDebInfo` build:

| Measure | Observed value |
|---|---:|
| Samples | 102 |
| Callback latency, mean | 13.178 ms |
| Callback latency, p95 | 14.107 ms |
| Processing frequency, mean | 10.003 Hz |
| Point reduction, mean | 6.744% |
| Clusters per frame | 3.0 |
| Confirmed tracks per frame | 3.0 |
