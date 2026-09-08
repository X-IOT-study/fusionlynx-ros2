# FusionLynx Architecture

This document owns the system-level architecture formerly described in the
root `AGENTS.md`. The complete pre-split text remains in `docs/AGENTS.full.md`.

## Pipeline

```text
MotionCore firmware
    -> motioncore-client / SDK
    -> ROS 2 hardware driver
    -> robot description / TF
    -> wheel odometry + FAST-LIO
    -> state_fusion_backend
    -> fused /odom
    -> RTAB-Map
    -> evaluation and thesis artifacts
```

## Responsibilities

- MotionCore firmware owns motor control, sensors, and the wire protocol.
- `motioncore-client` owns host-side protocol and SDK behavior.
- `motioncore_driver` owns ROS 2 hardware integration and lifecycle safety.
- `wheel_kinematics` converts encoder data into wheel odometry.
- FAST-LIO owns local lidar-inertial motion estimation.
- `state_fusion_backend` owns authoritative fused local odometry.
- RTAB-Map owns mapping, loop closure, map products, and `map -> odom`.
- `fusionlynx_description` owns robot geometry and sensor extrinsics.
- `fusionlynx_mapping` owns RTAB-Map integration and ICP loop-closure wiring.
- `fusionlynx_bringup` composes the integrated PC-side system.

Do not introduce a second handwritten mapper, production localization path,
MotionCore wire-protocol implementation, or `/odom` publisher without an
explicit architecture decision.

## Topic contracts

```text
/cmd_vel
/motioncore/encoder_raw
/motioncore/imu/data_raw
/motioncore/imu/mag_raw
/motioncore/wheel_odom
/motioncore/imu/data
/motioncore/imu/mag
/motioncore/diagnostics
/unilidar/cloud
/unilidar/imu
/fast_lio/odom
/fast_lio/cloud_registered_body
/odom
```

Do not silently rename, repurpose, or change frame semantics. If an interface
must change, update all publishers, subscribers, launch files, YAML files,
tests, documentation, and Research Vault records together.

## Control path

```text
joy -> game_controller_node -> teleop_twist_joy -> /cmd_vel
-> motioncore_driver -> MotionCore SDK -> firmware
```

Confirmed 8BitDo mapping:

```text
left stick vertical   -> linear.x
left stick horizontal -> angular.z
right stick horizontal -> linear.y
LB -> deadman; RB -> turbo
```

LB must enable motion only while held; releasing LB must stop immediately.
Do not bypass ROS 2 with raw serial teleoperation.
