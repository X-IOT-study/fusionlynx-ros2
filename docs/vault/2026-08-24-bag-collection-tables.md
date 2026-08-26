---
created: 2026-08-24
updated: 2026-08-24
status: implemented
tags: [experiment, rosbag2, data, timestamp, fusion]
---

# FusionLynx 测试 bag 采集与表格

## 目的

为融合矩阵 A-G 的每个场景、每个重复实验建立可追溯的 rosbag2 采集记录。
采集脚本不改变 ROS Header measurement timestamp；它同时保存 bag storage
timestamp 与 Header timestamp 的偏移，便于识别树莓派传输延迟、队列积压和
主机/设备时钟问题。

## 表格设计

`experiment_runs.csv` 是实验设计表，每次采集一行，记录 profile、场景、重复
次数、固件/SDK、ROS 发行版、RMW、主机和时间同步方法。

`bag_topic_stats.csv` 是 topic 统计表，每个 bag 的每个 topic 一行，记录消息
数、频率、间隔分位数、Header 倒退和
`bag_timestamp - header_timestamp` 的统计值，同时保留 `frame_id`、
`child_frame_id` 和 Header 覆盖率。

`bag_quality.csv` 是验收表，每次采集一行，记录必需 topic 是否进入 bag、时长、
时间戳单调性、偏移阈值、硬件运行验证和回放验证。`quality_status` 只表示
自动检查结果，不替代实际轨迹、TF 和建图验收。

## 当前 topic 集合

- A / wheel：`/motioncore/wheel_odom` 及 MotionCore 原始数据、TF、控制输入；
- B / LIO：`/fast_lio/odom`、`/unilidar/cloud`、`/unilidar/imu` 和 FAST-LIO
  registered body cloud；
- C-G / full：wheel、FAST-LIO、`/odom`、TF 和诊断。

`/Odometry`、`/path` 没有作为生产融合合同的默认 topic。若现场驱动仍发布
旧接口，用 `--extra-topic /Odometry --extra-topic /path` 显式加入，并在备注
中记录。

## 运行原则

1. 先启动实际链路，再运行采集器。
2. 每个场景按相同动作规程至少重复三次。
3. 录制完成后检查 `bag_quality.csv`，再进行 rosbag 回放和 TF/RTAB-Map 验收。
4. `hardware_runtime_verified` 和 `replay_verified` 默认是 false，不能因为
   bag 文件存在而自动标记为已验证。
5. wheel、FAST-LIO 和融合输出应分别保留自己的 Header timestamp；表格中的
   bag/Header 偏移用于诊断，不能用来覆盖原始测量时间。

实际 bag 大文件建议输出到 `/home/dream/rosbags/fusionlynx` 或 Vault 的
`05_Data/bags`，不要把运行生成物提交到源码目录。
