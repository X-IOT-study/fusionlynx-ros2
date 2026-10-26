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

## 从 RTAB-Map 数据库导出轨迹

导出脚本以只读方式读取数据库，导出的 TUM/CSV 轨迹只用于分析和可视化，
不会保留 RTAB-Map 的工作记忆、扫描数据和图连接关系。因此，导出轨迹
不能作为 RTAB-Map 的定位数据库。

`Node.pose` 可以导出为 evo 支持的 TUM 轨迹和便于检查的 CSV。数据库以只读
方式打开，默认输出到工作区的 `data/path/`：

```bash
python3 scripts/export_rtabmap_trajectory.py \
  /tmp/fusionlynx_20260826_1020.db \
  --output-dir data/path \
  --force
```

输出包括：

```text
data/path/fusionlynx_20260826_1020_estimate.tum
data/path/fusionlynx_20260826_1020_estimate.csv
data/path/fusionlynx_20260826_1020_trajectory_report.json
data/path/fusionlynx_20260826_1020_trajectory_report.txt
```

只有数据库中存在非全零的有效 `Node.ground_truth_pose` 时，脚本才会额外
生成 `*_ground_truth.tum`。当前 RTAB-Map 数据库若该字段全为零，不能把它
当作外部真值。

## 多轨迹 evo 可视化

轨迹使用 `LABEL=PATH` 传入，脚本会显式调用固定的 evo 虚拟环境，避免
混用系统 Python、用户目录 Matplotlib 和 venv 依赖：

```bash
python3 scripts/compare_trajectories_evo.py \
  "profile-E=data/path/fusionlynx_20260826_1020_estimate.tum" \
  "fast-lio=/tmp/fusionlynx_20260826_1020_estimate.tum" \
  --output-dir data/path/plots \
  --prefix fusionlynx_20260826_compare \
  --plot-mode xy
```

默认使用 `qtagg` 后端并打开 GUI 交互窗口。关闭窗口后命令才会结束，
同时仍会保存轨迹、XYZ、RPY 和速度图。脚本会把 evo 的用户配置和
Matplotlib 缓存写入 `/tmp`，不会修改系统 Python 包。

无桌面环境或通过 SSH 使用时，显式关闭 GUI 并使用非交互后端：

```bash
python3 scripts/compare_trajectories_evo.py \
  "profile-E=data/path/fusionlynx_20260826_1020_estimate.tum" \
  "fast-lio=/tmp/fusionlynx_20260826_1020_estimate.tum" \
  --no-show \
  --backend Agg
```

## 自动运行 profile 实验

以下脚本启动 `fusionlynx_bringup pc.launch.py`，等待 bringup 初始化，
调用现有采集器录制 bag，停止时再导出 RTAB-Map 数据库轨迹。它不启动
MotionCore 硬件驱动；硬件和传感器话题必须已由部署环境提供。

```bash
source /opt/ros/humble/setup.bash
source /home/dream/ros2_ws/install/setup.bash

python3 scripts/run_fusion_profile_experiment.py \
  --profile E \
  --duration 60 \
  --use-rviz \
  --run-mapping
```

批量测试 `A` 到 `G`：

```bash
python3 scripts/run_fusion_profile_experiment.py \
  --all-profiles \
  --duration 60 \
  --no-use-rviz \
  --run-mapping \
  --stop-on-error
```

每次运行会将数据库、bringup/录包日志和元数据放在
`/tmp/fusionlynx_experiments/<run-name>/`，bag 及采集表放在
`~/rosbags/fusionlynx/<run-name>/`，导出的轨迹放在 `data/path/`。每个
profile 使用独立的 bag 子目录，避免 `C-G` 的 `full` 链路同名冲突。正式实验前可用
`--dry-run` 检查命令，不会启动 ROS：

```bash
python3 scripts/run_fusion_profile_experiment.py \
  --profile E --duration 60 --dry-run
```

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

要让 RTAB-Map 在重启后定位到这张地图，必须使用生成该数据库的完整
LiDAR RTAB-Map `.db`，而不是导出的 PCD 或轨迹文件，并显式启动定位模式：

```bash
ros2 launch fusionlynx_bringup pc.launch.py \
  mapping_database_path:=/absolute/path/fusionlynx_lidar_map.db \
  localization:=true \
  initial_pose:="x y z roll pitch yaw"
```

数据库至少应包含当前点云链路对应的 `Node.pose`、图连接和 `Data.scan`
记录。仅有 RGB-D 图像/深度记录的旧数据库不能用于当前纯 LiDAR 定位。
