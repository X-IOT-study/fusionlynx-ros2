# fusionlynx_slam

`fusionlynx_slam` is the top-level launch and configuration package for the
FusionLynx SLAM system. It composes the following independent runtime packages:

```text
fast_lio
    LiDAR-inertial odometry frontend
    /fast_lio/odom

state_fusion_backend
    Wheel + FAST-LIO relative pose + planar-prior fixed-lag factor-graph backend
    MotionCore IMU/MAG are currently LIO-health inputs
    /odom
    odom -> base_link

RTAB-Map
    Mature raw-cloud/fused-odometry 3D mapping backend
    map -> odom

fusionlynx_mapping
    RTAB-Map integration launch package
    /rtabmap/cloud_map
    /rtabmap/map
```

The package boundary is intentional. `fusionlynx_slam` is the system entry
point, while the estimation and RTAB-Map mapping/localization responsibilities
remain independently testable.

## Launch

The default launch starts FAST-LIO without its RViz window, the state fusion
backend, and the RTAB-Map mapping backend:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fusionlynx_slam slam.launch.py
```

Useful arguments:

```bash
ros2 launch fusionlynx_slam slam.launch.py \
  run_fast_lio:=false \
  run_state_fusion:=false

ros2 launch fusionlynx_slam slam.launch.py \
  fast_lio_config_file:=unilidar_L2.yaml \
  fast_lio_rviz:=false

ros2 launch fusionlynx_slam slam.launch.py \
  run_mapping:=false
```

To localize against a saved RTAB-Map LiDAR database:

```bash
ros2 launch fusionlynx_slam slam.launch.py \
  mapping_database_path:=/absolute/path/fusionlynx_lidar_map.db \
  localization:=true \
  initial_pose:="x y z roll pitch yaw"
```

## FAST-LIO to base_link validation

Before evaluating wheel/LIO fusion, run the isolated FAST-LIO chain:

```bash
ros2 launch fusionlynx_slam fast_lio_base_test.launch.py \
  use_rviz:=true
```

This launch does not start `state_fusion_backend` and does not publish the
production `/odom` or `odom -> base_link`. It converts FAST-LIO's
`camera_init -> lidar_link` observation using the runtime TF
`base_link <- lidar_link` from `robot_state_publisher`, resets the initial
pose to zero, and publishes only:

```text
/fast_lio/base_odom
fast_lio_odom -> base_link
```

RTAB-Map consumes `/fast_lio/base_odom` and
`/fast_lio/cloud_registered_body`. The static robot description remains the
authority for `base_link -> lidar_link`, `base_link -> body`, and
`lidar_link -> unilidar_lidar`.

The bridge deliberately does not declare a second
`lidar_to_base_xyz/rpy` parameter. The inverse mounting transform for this
validation path comes from the URDF TF tree. The fusion backend's explicit
conversion parameters are not used by this launch and must be reconciled with
the same TF contract before fusion is re-enabled.

Validate the isolated chain before enabling fusion:

```bash
ros2 topic echo /fast_lio/odom --once
ros2 topic echo /fast_lio/base_odom --once
ros2 run tf2_ros tf2_echo fast_lio_odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link body
ros2 topic hz /fast_lio/base_odom
```

## Runtime prerequisites

This launch file is the SLAM processing entry point. It does not start the
MotionCore hardware bridge, the UniLidar driver, or `robot_state_publisher`.
Start those layers first so `/unilidar/cloud`, `/unilidar/imu`, the MotionCore
factor inputs, and the static `base_link -> lidar_link -> unilidar_lidar` and
`base_link -> body` TF chains already exist:

```bash
ros2 launch motioncore_bringup motioncore_bringup.launch.py
ros2 launch unitree_lidar_ros2 launch.py publish_tf:=false
ros2 launch fusionlynx_description display.launch.py use_rviz:=false
ros2 launch fusionlynx_slam slam.launch.py
```

On a split Raspberry Pi/PC deployment, run the hardware launches on the
Raspberry Pi and the description plus this SLAM launch on the PC. The exact
hardware bringup command may be replaced by the deployment-specific
`motioncore_bringup` and UniLidar launch files.

The default system profile is installed at
`config/fusionlynx_slam.yaml`. The child packages can still be launched
individually with their own parameter files.

## Responsibility boundary

FAST-LIO's `/fast_lio/*` topics are frontend observations and debug outputs.
They are not the final `odom -> base_link` TF authority and its local map is
not treated as the final global map. The default RTAB-Map wrapper consumes the
deskewed `/fast_lio/cloud_registered_body`, fused `/odom`, and the robot TF
tree; it owns the formal
mapping/database path, ICP loop closure, localization, and `map -> odom`. The
`fusionlynx_mapping` package is only the RTAB-Map integration layer; it does
not contain a second handwritten mapping implementation. Mapping and
localization remain outside `state_fusion_backend`.

The mature mapping dependency is `rtabmap_launch` from
`ros-humble-rtabmap-ros`. It must be installed before the default launch can
resolve `fusionlynx_mapping/mapping.launch.py`. The wrapper enables approximate
synchronization for `/odom` and `/fast_lio/cloud_registered_body` because the
fused odometry and deskewed LiDAR scan are published by independent nodes and
should not be expected to carry identical header timestamps. The default
synchronization interval is `0.10 s`, with
per-topic and synchronization queues of `30`; these are launch arguments of
`fusionlynx_mapping/mapping.launch.py`.

The wrapper's `database_path` and `localization` arguments must be used
together when restarting on an existing map. A saved `.pcd` or exported TUM/CSV
trajectory is useful for inspection, but it does not contain the RTAB-Map
working-memory graph and scan records needed for localization.

The top-level launch forces FAST-LIO's `publish.tf_en` to `false`. Therefore
the expected dynamic TF is only `odom -> base_link`; FAST-LIO still publishes
`/fast_lio/odom` as a topic-level observation whose pose is expressed at
`lidar_link`.
