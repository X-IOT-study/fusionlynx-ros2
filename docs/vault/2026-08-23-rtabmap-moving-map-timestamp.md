---
created: 2026-08-23
updated: 2026-08-23
status: investigated
tags: [slam, rtabmap, odom, timestamp, fast-lio]
---

# RTAB-Map 移动时地图叠加调查

## 结论

地图在机器人移动或旋转时出现叠加，主要原因是 `/odom` 混用了两个时间域。
`state_fusion_backend` 的 LIO 更新使用 `/fast_lio/odom` 的时间戳，而
wheel prediction 使用 `/motioncore/wheel_odom` 的时间戳。运行时观测到
wheel 原始编码器、wheel odom 和部分 `/odom` 时间戳领先当前 ROS 时间约
`1.3~1.5 s`；FAST-LIO 和点云仍在当前 ROS 时间附近。

因此 `/odom` 会在 LIO 时间和未来约 1.5 s 的 wheel 时间之间跳变。RTAB-Map
按时间把 `/odom` 与 `/unilidar/cloud` 配对，移动时会把同一帧点云变换到错误
的机器人姿态，表现为旋转叠加、重影和地图变形。静止时姿态变化小，所以现象
不明显。

## 证据

- `/fast_lio/odom` 延迟约 `+0.06 s`。
- `/unilidar/cloud` 延迟约 `+0.10~0.15 s`。
- `/motioncore/encoder_raw` 延迟约 `-1.35 s`。
- `/motioncore/wheel_odom` 延迟约 `-1.37 s`。
- 同进程采样中，`lio-cloud` 通常约 `0.04 s`，但 `odom-cloud` 会出现约
  `1.5 s` 的超前；另一些 `/odom` 样本与 LIO 时间相同。
- TF 查询确认 `odom -> base_link` 的动态源为融合节点；`odom_wheel`、
  `camera_init` 和 UniLidar 诊断 TF 当前未形成额外有效父链。

## 修复

- `motioncore_driver` 新增 `use_device_timestamps`，默认 `false`，原始
  编码器和 IMU ROS 时间戳使用主机接收时间。
- `state_fusion_backend` 新增 `max_future_stamp: 0.25`，拒绝明显超前的
  wheel odom 输入，避免未来时间进入 `/odom`。
- 保留设备时间映射作为显式实验选项，不能在未完成时钟标定时用于 RTAB-Map
  主链路。
- RTAB-Map 不再默认消费原始 `/unilidar/cloud`。FAST-LIO 已发布的
  `/fast_lio/cloud_registered_body` 是去畸变后的 body-frame 当前扫描，避免
  机器人旋转时把单帧内部运动畸变写入地图。
- `robot_description` 增加 `base_link -> body` 静态 TF，使用
  `state_fusion_backend` 中 body 与 base 的固定外参。FAST-LIO的
  `/fast_lio/odom` 观测位姿使用 `camera_init -> lidar_link`，融合节点使用
  `T_lidar_base` 转换到最终 base 状态。不得把
  `/fast_lio/cloud_registered` 或 `/fast_lio/local_map` 再按 `/odom` 变换，
  因为它们已经位于 `camera_init` 全局坐标系。

## 验证要求

重启完整运行链后重新检查：

```bash
ros2 topic delay /motioncore/encoder_raw
ros2 topic delay /motioncore/wheel_odom
ros2 topic delay /odom
ros2 topic delay /fast_lio/odom
```

目标是 `/motioncore/encoder_raw`、`/motioncore/wheel_odom` 和 `/odom` 不再
出现持续负延迟；然后使用全新 RTAB-Map 数据库测试直线、横移和原地旋转。
重点确认 `ros2 run tf2_ros tf2_echo base_link body` 有效，并确认 RTAB-Map
日志显示订阅 `/fast_lio/cloud_registered_body`。
