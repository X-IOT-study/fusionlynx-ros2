# FAST-LIO and RTAB-Map

This document owns the LIO and mapping integration rules. The complete
pre-split text remains in `docs/AGENTS.full.md`.

## FAST-LIO

FAST-LIO is the primary LIO frontend. ORB-SLAM3 is comparison-only. The
integrated system treats `/fast_lio/odom` as a frontend observation and keeps
FAST-LIO dynamic TF publication disabled when it would compete with the fusion
backend.

At the fusion boundary, verify frame IDs, timestamps, covariances, pose
direction, and the sensor-to-base extrinsic. Preserve the established topics:

```text
/unilidar/cloud
/unilidar/imu
/fast_lio/odom
/fast_lio/cloud_registered_body
```

## RTAB-Map

RTAB-Map consumes:

```text
/odom
/fast_lio/cloud_registered_body
```

It owns mapping, loop closure, map products, and `map -> odom`; it must not
publish authoritative `/odom`. Use the cloud representation intended for
RTAB-Map. Do not transform an already world/global-registered cloud a second
time as though it were still in a body or sensor frame. Preserve approximate
synchronization for independently produced odometry and clouds.

For an existing database, explicitly select localization mode:

```bash
ros2 launch fusionlynx_bringup pc.launch.py \
  mapping_database_path:=/absolute/path/fusionlynx_lidar_map.db \
  localization:=true \
  initial_pose:="x y z roll pitch yaw"
```

`mapping_database_path` (forwarded to RTAB-Map as `database_path`) selects the
SQLite file; it does not select mapping versus localization. The database must
contain scans matching the current LiDAR input contract. PCD and TUM/CSV trajectories are analysis outputs and cannot
replace the RTAB-Map graph database.

Keep RTAB-Map integration inside `fusionlynx_mapping`.
