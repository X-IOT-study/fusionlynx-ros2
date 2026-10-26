# fusionlynx_bringup

`fusionlynx_bringup` provides the PC-side one-command entry point for the
FusionLynx model, state fusion, FAST-LIO, and RTAB-Map stack.

## PC launch

Start the hardware/data-collection side first. Then run one command on the PC:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fusionlynx_bringup pc.launch.py
```

This launches:

```text
fusionlynx_description
    robot_state_publisher
    encoder-driven joint state publisher
    base_link -> lidar_link -> unilidar_lidar
    RViz2

fusionlynx_slam
    FAST-LIO -> /fast_lio/odom
    state_fusion_backend -> /odom and odom -> base_link
    RTAB-Map -> map -> odom
```

To restart on an existing RTAB-Map LiDAR map, use localization mode
explicitly:

```bash
ros2 launch fusionlynx_bringup pc.launch.py \
  mapping_database_path:=/absolute/path/fusionlynx_lidar_map.db \
  localization:=true \
  initial_pose:="x y z roll pitch yaw"
```

`mapping_database_path` alone does not load the database as a fixed map. With
`localization:=true`, RTAB-Map opens the existing graph with incremental memory
disabled and publishes the `map -> odom` correction while
`state_fusion_backend` continues to publish `odom -> base_link`.

FAST-LIO's standalone RViz is disabled. The FusionLynx description RViz is the
only RViz window started by this launch.

## TF authority

```text
map -> odom             RTAB-Map
odom -> base_link       state_fusion_backend
base_link -> sensors    robot_state_publisher/static TF
```

FAST-LIO estimates an IMU/body state internally, but publishes
`/fast_lio/odom` as the physical LiDAR pose with
`camera_init -> lidar_link` message frames. It does not publish a dynamic TF in
this stack. `state_fusion_backend` applies the configured lidar-to-base
extrinsic before publishing the final `odom -> base_link`.

## Deployment boundary

`motioncore_bringup` remains the MotionCore-layer entry point and should be
started on the Raspberry Pi or the host connected to the MotionCore device.
The UniLidar driver is also started according to the deployment layout. This
PC launch intentionally does not start either hardware driver a second time.
