---
created: 2026-08-24
updated: 2026-08-24
status: implemented
tags: [fast-lio, tf, base-link, validation]
---

# FAST-LIO 到 base_link 隔离验证链

## 决策

在重新启用 `state_fusion_backend` 之前，先使用
`fusionlynx_slam/launch/fast_lio_base_test.launch.py` 验证：

```text
UniLidar L2 -> FAST-LIO -> camera_init -> lidar_link
                                      |
                                      v
                           base_link <- lidar_link
                                      |
                                      v
                         fast_lio_odom -> base_link
```

桥接节点不再接收独立的 `lidar_to_base_xyz/rpy` 数组，而是在每个有效
FAST-LIO 位姿时间戳查询 `base_link <- lidar_link`。物理安装外参因此只由
`fusionlynx_description` 的 URDF/robot_state_publisher 提供。

## 已验证

- 安装空间中的 xacro 能生成 `base_link -> body` 和
  `base_link -> lidar_link`。
- `ros2 launch ... --show-args` 能解析隔离 launch，并显示
  `tf_lookup_timeout_sec`。
- 无硬件隔离 launch 在关闭 RViz 和 mapping 时能启动
  `robot_state_publisher`、FAST-LIO 和桥接节点；FAST-LIO 日志确认
  `odom_child_frame=lidar_link` 且 `publish_tf=false`。
- 两帧伪造 `/fast_lio/odom` 通过本地静态 TF 后，桥接输出
  `fast_lio_odom -> base_link`；首帧归零，第二帧输出
  `(0.390731, 0.920505, 0)`，与 `T_base_lidar` 的旋转一致。

## 实机验收

启动 MotionCore、UniLidar L2 后运行：

```bash
ros2 launch fusionlynx_slam fast_lio_base_test.launch.py \
  use_rviz:=true
```

依次检查：

```bash
ros2 topic echo /fast_lio/odom --once
ros2 topic echo /fast_lio/base_odom --once
ros2 run tf2_ros tf2_echo fast_lio_odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link body
ros2 topic hz /fast_lio/base_odom
```

确认直行、横移、原地旋转的方向和尺度后，才允许将同一 TF 合同用于
`state_fusion_backend` 和正式 RTAB-Map 建图。当前构建与伪造数据测试不等价
于真实小车运动验收。
