# Deployment and Runtime Operations

This document owns lifecycle, build, bringup, and runtime debugging guidance.
The complete pre-split text remains in `docs/AGENTS.full.md`.

## Environment and build

ROS 2 distribution: Humble.

```bash
source /opt/ros/humble/setup.bash
source /home/dream/ros2_ws/install/setup.bash
cd /home/dream/ros2_ws
colcon build --symlink-install --packages-up-to <package_name>
source install/setup.bash
```

Use a full workspace build only when integration validation requires it. After
changing launch files, YAML, URDF/Xacro, or installed resources, rebuild and
source the overlay; do not assume `ros2 launch` uses the source-tree copy.

Focused tests:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
colcon test --packages-select state_fusion_backend
```

## Lifecycle safety

`motioncore_driver` is a lifecycle node:

```text
configure -> prepare serial / SDK / publishers
activate  -> start live communication and publishing
deactivate -> stop motion and release active resources
```

Lifecycle state is a safety boundary. Check it before changing motion-control
code or diagnosing a robot that does not move.

## Bringup

```bash
ros2 launch fusionlynx_bringup pc.launch.py
```

The integrated PC-side bringup composes robot description, FAST-LIO,
state-fusion, RTAB-Map, and RViz. Hardware drivers may need to run separately.
For isolated testing:

```bash
ros2 launch fusionlynx_slam slam.launch.py
ros2 launch fusionlynx_mapping mapping.launch.py
```

`slam.launch.py` does not start `robot_state_publisher` or
`fusionlynx_description`. Start the description first so the static
`base_link -> lidar_link` and `base_link -> body` transforms exist.

Do not alter integrated bringup to hide a subsystem defect.

## Runtime checks

```bash
ros2 topic info -v /odom
ros2 topic hz /odom
ros2 topic echo /fast_lio/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

For topic and sensor failures, check topic existence, QoS compatibility,
timestamp, frame ID, TF availability, then data content. For hardware, check
lifecycle state, serial device, permissions, baudrate, device ID, auto-report,
and ROS topic output in that order.
