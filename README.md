# FusionLynx ROS 2

FusionLynx ROS 2 是 MotionCore 麦克纳姆机器人研究项目的总集成仓库，
负责机器人描述、FAST-LIO、状态融合、建图、bringup、实验脚本和研究文档。

## 仓库关系

```text
fusionlynx-ros2
├── src/fusionlynx/       项目自有 ROS 2 包
├── src/motioncore/       motioncore-ros2 一方子模块
├── src/FAST_LIO/         FAST-LIO 固定版本子模块
├── src/livox_ros_driver2/固定版本子模块
├── src/orbslam3_ros/     固定版本子模块
└── src/ros2_astra_camera/固定版本子模块
```

MotionCore 固件和主机端 SDK 使用独立仓库维护：

```text
motioncore-firmware
motioncore-client
```

## 获取源码

```bash
git clone https://github.com/X-IOT-study/fusionlynx-ros2.git
cd fusionlynx-ros2
git submodule update --init --recursive
```

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to state_fusion_backend
source install/setup.bash
```

完整 Git 管理规则见
`docs/development/git_workflow.md`。原始 rosbag、点云、数据库和构建输出
不进入 Git，实验记录维护在 Research Vault。
