This is a ROS2 package for Unitree Lidar L2.

## TF publishing policy

The driver publishes the UniLidar cloud and IMU topics, but its diagnostic TF
chain is disabled by default with `publish_tf:=false`.

FusionLynx uses `lidar_link` as the canonical UniLidar L2 coordinate frame. The
`fusionlynx_description` package owns the identity relation:

```text
base_link -> lidar_link -> unilidar_lidar
```

This prevents `unilidar_lidar` from receiving a second parent from the driver's
`unilidar_imu -> unilidar_lidar` diagnostic transform. The driver TF chain can
be enabled for standalone visualization:

```bash
ros2 launch unitree_lidar_ros2 launch.py publish_tf:=true
```

Do not enable that option at the same time as the FusionLynx model launch. The
LiDAR/IMU extrinsic remains the FAST-LIO configuration value and is not used as
a conversion between `lidar_link` and `unilidar_lidar`, because those two frames
use the same UniLidar L2 axes.
