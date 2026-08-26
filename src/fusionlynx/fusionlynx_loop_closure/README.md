# fusionlynx_loop_closure

This package reserves the loop-closure and global-optimization boundary for
the FusionLynx SLAM system.

It is intentionally not an algorithm implementation yet. The planned
interface is:

```text
rtabmap/mapPath or RTAB-Map graph data
    -> loop closure / global optimizer
    -> /fusionlynx_slam/optimized_path
```

RTAB-Map owns loop closure and publishes its map graph/path outputs. Until a
verified FusionLynx loop-closure adapter exists, this package does not publish
`/fusionlynx_slam/optimized_path` and does not modify the fused odometry TF.

The package can be enabled from the top-level launch to verify composition:

```bash
ros2 launch fusionlynx_slam slam.launch.py run_loop_closure:=true
```
