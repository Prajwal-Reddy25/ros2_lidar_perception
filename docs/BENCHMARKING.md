# Benchmarking and evidence

Build in `Release` or `RelWithDebInfo`, source the overlay, then run:

```bash
ros2 run lidar_perception run_benchmark.py \
  --warmup 3 --duration 30 --output results/benchmark.json
```

The tool launches the seeded synthetic workload unless `--no-launch` is given.
It records every typed metrics message after warm-up and reports distributions
for callback latency and completion rate, point reduction, cluster count,
confirmed track count, and adjacent-frame track-ID retention. Results are JSON
so CI or a lab notebook can archive them without scraping logs.

Latency is measured inside the subscriber callback, from immediately before
PointCloud2 conversion through the debug-cloud and object publication calls. It
excludes metrics/diagnostics publication, DDS transport, and queue residence
time. `processing_hz` is an exponential moving estimate of completed callbacks;
under the default workload it is driven by the 10 Hz synthetic input. It is an
observed input/workload rate, not a maximum-throughput measurement. Use latency
percentiles for compute comparisons and an external end-to-end timestamp probe
for product-level deadlines.

Synthetic results prove repeatable execution and regression behavior, not
real-world accuracy. Report the CPU, build type, middleware, parameters, source
dataset, warm-up, duration, and sample count with any number. Never compare runs
with different input density or output subscribers as if they were equivalent.

If a checked-in evidence report exists under `docs/evidence`, it was generated
on the stated host by this tool. It is not a performance guarantee.
