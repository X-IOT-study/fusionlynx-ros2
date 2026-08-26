# FusionLynx Git 管理规范

更新时间：2026-08-26

## 1. 根仓库职责

根仓库是 FusionLynx ROS 2 集成仓库，负责保存能够复现系统集成状态的
项目内容：

```text
src/fusionlynx/              项目自有 ROS 2 包
src/motioncore/              一方 MotionCore ROS 2 开发仓库
src/motioncore-sdk-v1.0.0/   当前使用的 SDK 快照及其许可证
src/unitree_lidar_ros2/      当前工作区使用的驱动快照
docs/                        设计、验证记录和论文源文件
scripts/                     可复用分析与实验脚本
AGENTS.md                    工作区稳定约束
```

构建输出、缓存、原始数据和工具运行目录不属于根仓库。

## 2. 外部仓库边界

以下目录是外部 Git 仓库，根仓库只记录子模块提交号：

```text
src/FAST_LIO
src/livox_ros_driver2
src/orbslam3_ros
src/ros2_astra_camera
```

其远程地址和路径记录在根目录 `.gitmodules`。更新外部仓库时必须固定到
经过验证的提交，而不是让根仓库依赖外部仓库的浮动分支。

`src/motioncore` 是一方开发仓库，不是可忽略的本地缓存。它包含
`imu_preprocess`、`motioncore_driver`、`motioncore_msgs`、
`wheel_kinematics` 和 `motioncore_bringup`，这些代码必须在 MotionCore
仓库中独立提交和维护，根仓库通过子模块指针记录集成版本。

MotionCore 的正式远程仓库为：

```text
https://github.com/X-IOT-study/motioncore-ros2.git
```

根仓库通过 `.gitmodules` 引用该地址，MotionCore 的源码提交、分支和版本
发布仍在 `motioncore-ros2` 内完成。

## 3. 数据和论文材料

原始 rosbag、点云、RTAB-Map 数据库和 MinerU 中间产物不进入根仓库：

```text
build/ install/ log/
.mineru_output/
data/*.bag data/*.mcap data/*.db*
data/*.pcd data/*.ply
```

实验必须把以下内容分开保存：

```text
代码和参数       → 根仓库
实验配置与结果   → Research Vault/03_Experiment
原始数据         → Research Vault/05_Data 或外部数据盘
论文阅读笔记     → Research Vault/07_Reference
```

根仓库中的 `data/README.md` 和 `data/manifests/` 只用于说明数据来源、
文件哈希、采集时间和复现实验命令。

## 4. 分支和提交

`main` 只接受已经通过目标包构建、测试和必要运行验证的集成状态。

分支命名：

```text
feature/<area>-<short-name>
fix/<area>-<short-name>
refactor/<area>-<short-name>
experiment/<yyyymmdd>-<short-name>
```

提交采用简短的 Conventional Commits 风格：

```text
feat(state_fusion_backend): add factor provider
fix(state_fusion_backend): align recovered lio frame
refactor(state_fusion_backend): split factor construction
test(state_fusion_backend): cover recovery alignment
docs(vault): record fusion experiment decision
chore(repo): update submodule pin
```

一个提交只表达一个可审查的意图。算法、配置、实验结果和文档不要使用
一个无边界的“全部修改”提交混在一起。

## 5. 日常暂存规则

禁止在本工作区直接使用：

```bash
git add .
git add -A
```

推荐先查看变更，再按目录或文件白名单暂存：

```bash
git status --short --branch
git diff --stat
git diff --check
git add src/fusionlynx/state_fusion_backend
git add docs/development/git_workflow.md
git diff --cached --check
```

外部子模块和一方 MotionCore 子模块必须在子模块内提交后，根仓库只更新
其指针：

```bash
git -C src/motioncore status
git -C src/motioncore diff
git -C src/motioncore add imu_preprocess motioncore_driver motioncore_msgs \
  wheel_kinematics motioncore_bringup
git -C src/motioncore commit

git -C src/FAST_LIO status
git -C src/FAST_LIO log -1 --oneline
git add src/FAST_LIO
git add src/motioncore
```

不要把子模块内部未提交的修改误当成根仓库源码提交。根仓库只能记录
MotionCore 已提交的版本；当前工作副本中的未提交修改必须先在
`src/motioncore` 内审查和提交。

## 6. 研究实验提交

实验代码提交应至少包含：

```text
实验配置
运行命令
输入数据标识或哈希
软件提交号
结果文件位置
验收指标和解释
```

完整结果记录写入 Research Vault。根仓库只保存必要的配置模板、分析脚本、
小型示例和经过筛选的论文图表。

## 7. 历史大文件清理记录

总仓库初始提交曾误加入：

```text
src/unitree_lidar_ros2/.vscode/browse.vc.db
```

该 IDE 数据库约为 1.4 GB。仅修改 `.gitignore` 无法从 Git 历史中移除它，
因此在总仓库尚未成功推送到 GitHub 的前提下，已完成本地 `main` 历史清理。

清理后的约束如下：

1. `.vscode/`、`.marscode/`、构建目录和运行日志继续由 `.gitignore` 排除；
2. rosbag、数据库、点云和其他实验原始数据不提交到总仓库；
3. 清理后的 `main` 历史中不再包含 `browse.vc.db`；
4. 历史重写后产生的旧提交号不得继续用于总仓库协作。

推送清理后的 `main` 前，应先确认远程仓库没有已接受的旧历史。如果远程
已经存在旧的 `main`，必须先评估是否需要显式的强制更新，不能直接覆盖。

总项目根仓库的正式远程地址为：

```text
https://github.com/X-IOT-study/fusionlynx-ros2.git
```

仓库治理后的提交顺序为：

1. 在 `motioncore-ros2` 内提交 MotionCore 代码；
2. 在 `fusionlynx-ros2` 内提交项目代码、规则和子模块指针；
3. 在干净克隆中验证 `.gitmodules`、构建和测试；
4. 将两个本地提交分别推送到对应远程；
5. 通过受保护的 `main` 和审查分支继续开发。

当前四个外部子模块以及一方 `src/motioncore` 工作副本均有本地修改。
不得在未保存这些修改前执行 `git submodule update --init --recursive`
或切换其提交。
