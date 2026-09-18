# Algorithms and tuning

## Preprocessing

Non-finite points are removed first. A PCL voxel grid replaces points in each
cubic leaf with a centroid. The crop box then rejects points outside `[min,max]`
in x/y/z. These early reductions bound downstream k-d-tree and RANSAC cost.

An outlier stage is optional:

- `statistical` rejects points whose mean neighbor distance exceeds the global
  mean by the configured standard-deviation multiplier. It is useful for dense,
  approximately uniform scans but costs neighbor queries.
- `radius` requires a minimum number of neighbors within a fixed radius. It is
  predictable for isolated snow/dust returns but can erase distant sparse data.
- `none` is the default because synthetic and many already-conditioned datasets
  do not benefit enough to justify the cost.

## Ground segmentation

PCL RANSAC fits a plane constrained to be approximately perpendicular to +Z.
`ground.max_angle_deg` prevents a wall from winning when it has more returns
than the road. `ground.distance_threshold` should exceed vertical measurement
noise and local road roughness, but remain below the smallest obstacle height
that matters. One global plane is intentionally a baseline: undulating roads
need patch-wise or polar ground segmentation.

## Clustering and boxes

Euclidean clustering performs radius-connected components over a k-d tree.
Tolerance should grow with range for nonuniform spinning LiDAR; this fixed
tolerance implementation works best over a bounded ROI. Minimum/maximum point
counts suppress speckle and pathological components.

Axis-aligned boxes use XYZ extrema. Oriented boxes compute the principal XY
axis, project all cluster points into that yaw frame, take extrema there, and
rotate the center back. This is a yaw-only PCA box, appropriate for road users;
it is not a minimum-volume 3D box. Nearly circular clusters have ambiguous yaw.

## Tracking

Each track models constant planar velocity. Prediction uses white-acceleration
process noise; correction observes x/y centroids. Association constructs all
gated track/detection distances, sorts them, then greedily takes nonconflicting
pairs. This is deterministic and adequate at modest object counts, but crowded
scenes may warrant Hungarian/JV assignment with size, yaw, and class costs.

Track IDs are process-local and reset on node restart or any accepted runtime
parameter update. Velocity is reported only for confirmed tracked objects.

## Tuning order

1. Set the ROI in the cloud frame and inspect `~/filtered`.
2. Choose voxel size against smallest required obstacle detail.
3. Tune plane angle and distance while inspecting ground/nonground clouds.
4. Tune cluster tolerance and point-count limits by range and sensor density.
5. Select AABB/OBB and only then tune association distance and tracker noise.
6. Benchmark representative recorded sequences, not just the synthetic scene.
