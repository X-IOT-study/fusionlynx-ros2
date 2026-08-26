# FusionLynx 数据采集脚本

## 采集测试 bag

在已经启动 MotionCore、UniLidar、FAST-LIO 和融合链路的终端中运行：

```bash
source /opt/ros/humble/setup.bash
source /home/dream/ros2_ws/install/setup.bash

python3 /home/dream/ros2_ws/scripts/collect_fusion_test_bag.py \
  --experiment-id exp010 \
  --fusion-profile E \
  --scene lineturn \
  --repeat-index 1 \
  --duration 60 \
  --output-root /home/dream/rosbags/fusionlynx \
  --config-file src/fusionlynx/state_fusion_backend/config/state_fusion_backend.yaml \
  --launch-command "ros2 launch fusionlynx_bringup pc.launch.py fusion_profile:=E" \
  --time-sync-method host_receive_time
```

`A` 选择 wheel-only，`B` 选择 LIO-only，`C-G` 选择 wheel + FAST-LIO +
融合输出。可用 `--chain raw|wheel|lio|full|compare` 覆盖录制集合。

`--dry-run` 只打印 run ID、bag 路径、topic 集合、命令和表路径，不访问 ROS
graph，也不创建文件。录制前会尽力执行 `ros2 topic list -t` 并记录现场快照；
默认只警告缺失 topic，`--strict-topic-check` 才会在录制前中止。

## 输出结构

```text
<output-root>/
├── bags/
│   └── <chain>-<scene>-<parameter-version>-<YYYYMMDD>-<seq>.bag/
│       ├── collection_manifest.json
│       ├── bag_info.txt
│       └── topic_stats.json
├── manifests/
│   └── <run-id>.json
└── tables/
    ├── experiment_runs.csv
    ├── bag_topic_stats.csv
    └── bag_quality.csv
```

三个 CSV 可以直接导入 Excel、LibreOffice 或 pandas：

| 表 | 粒度 | 用途 |
| --- | --- | --- |
| `experiment_runs.csv` | 每次采集一行 | 实验条件、版本、时间同步和 bag 路径 |
| `bag_topic_stats.csv` | 每个 bag 的每个 topic 一行 | 消息数、频率、间隔、frame 语义和 bag/Header 时间偏移 |
| `bag_quality.csv` | 每次采集一行 | 必需 topic、时间戳、时长、回放和硬件验收 |

`frame_id`、`child_frame_id` 和 `header_coverage` 用于检查 odometry/TF 的坐标
语义与时间戳覆盖率。`bag_minus_header_*` 是 bag storage timestamp 减 ROS
Header measurement timestamp。它用于观察传输/排队延迟，不会修改 bag 中的原始 Header 时间。
出现持续负值时，应先检查主机/设备时间映射，不要在后处理表格中把它改成
零。

## 现有离线分析器

已有的 `motioncore_analyze_bag.py` 仍可单独使用：

```bash
python3 scripts/motioncore_analyze_bag.py <bag-directory> \
  --json-out <bag-directory>/topic_stats.json
```

采集器没有硬件运行或回放验证能力。只有在实际执行相应检查后，才传入
`--hardware-runtime-verified` 或 `--replay-verified`。

## 从 RTAB-Map 数据库导出 PCD

当 `rtabmap-export --cloud --scan` 对 FusionLynx 数据库报告 `0 points`
时，可直接读取 RTAB-Map 的 `Data.scan` 和 `Node.pose`：

```bash
python3 scripts/export_rtabmap_db_to_pcd.py \
  ~/.ros/fusionlynx_fast_lio_base_test.db
```

默认输出到 `data/PointClouds/`，并附加当前时间戳：

```text
data/PointClouds/<database>_global_YYYYMMDD_HHMMSS.pcd
data/PointClouds/<database>_scan_sensor_YYYYMMDD_HHMMSS.pcd
```

`*_global.pcd` 应用了数据库中的节点位姿和扫描外参，适合查看整张地图；
`*_scan_sensor.pcd` 保留各帧扫描的传感器局部坐标，适合检查原始点云。脚本
不修改数据库，也不依赖 ROS 节点。使用 `--no-timestamp` 禁止时间戳，
使用 `--no-local-output` 只导出全局点云。
