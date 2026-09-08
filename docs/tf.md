# TF and Sensor Extrinsics

This document owns TF publication and physical mounting rules. The complete
pre-split text remains in `docs/AGENTS.full.md`.

## Ownership

Each dynamic edge has exactly one authoritative publisher:

```text
map -> odom
    RTAB-Map

odom -> base_link
    state_fusion_backend

base_link -> sensor frames
    robot_state_publisher / static TF
```

FAST-LIO must not publish a competing `odom -> base_link` in the integrated
system. Do not create duplicate or disconnected TF trees.

Before changing TF, inspect the current authority:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

## Physical extrinsics

`fusionlynx_description` is the source of truth for robot geometry and sensor
placement. Do not guess extrinsics, replace them with identity transforms,
swap axes silently, duplicate mounting parameters, or compensate for an
incorrect URDF transform with an unrelated static TF.

When a downstream component needs an inverse or composed transform, derive it
mathematically from the authoritative extrinsic. When mounting changes:

1. Update the owning URDF/Xacro or configuration.
2. Verify the resulting TF.
3. Update dependent fusion transforms.
4. Test the affected sensor pipeline.

## Fusion boundary checks

For `/fast_lio/odom`, verify `header.frame_id`, `child_frame_id`, timestamp,
covariance, sensor-to-base extrinsic, and pose direction. Do not silently
reinterpret an unexpected frame contract.

For raw odometry debugging, set RViz Fixed Frame to `odom`, test forward,
lateral, and rotational motion separately, and verify TF before judging the
trajectory. Do not use a changing `map` frame to judge local odometry.
