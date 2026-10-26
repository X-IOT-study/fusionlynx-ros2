# 实验数据边界

本目录只保留数据说明和可复现实验所需的轻量元数据。

以下内容不进入 Git：

- rosbag、MCAP 和 RTAB-Map 数据库；
- 原始点云、地图和中间导出文件；
- 设备采集日志和临时诊断输出。

原始数据放在 Research Vault 的 `05_Data/` 或外部数据盘。实验记录放在
`03_Experiment/`，并记录数据路径、文件哈希、采集时间、使用的代码提交号
和复现命令。

`data/path/` 用于本地离线轨迹导出和可视化中间结果，不纳入 Git。数据库
轨迹由 `scripts/export_rtabmap_trajectory.py` 导出为 TUM/CSV，PNG 对比图
由 `scripts/compare_trajectories_evo.py` 生成。正式实验数据仍应保存到
Research Vault 的 `05_Data/` 或外部数据盘，并在实验记录中登记输入数据库、
轨迹文件、代码提交号和 evo 命令。
