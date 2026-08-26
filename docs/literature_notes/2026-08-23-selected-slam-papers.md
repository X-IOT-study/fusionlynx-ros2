# 2026-08-23 选定 SLAM 文献阅读笔记

## 说明

论文来源限定为 `/home/dream/Documents/docs/paper`。本批 PDF 先复制到
Vault 的 `07_Reference/papers/downloads/selected_2026-08-23/`，再使用
`/home/dream/Applications/MinerU/.venv/bin/mineru` 转换到
`/home/dream/ros2_ws/.mineru_output/selected_2026-08-23/`。

本文中的行号是 MinerU Markdown 文件的稳定阅读定位，不把本笔记替代原文。

## 1. Voxel-SLAM

来源：`Voxel-SLAM.pdf`，MinerU：
`.mineru_output/selected_2026-08-23/voxel_slam/voxel_slam/auto/voxel_slam.md`

### 关键内容

- 论文明确区分 LiDAR odometry 与 LiDAR SLAM：前者关注实时定位和累积地图，
  后者还要通过历史数据关联持续修正地图与轨迹（S011-S017）。
- 系统按短期、中期、长期和多地图关联划分职责，分别对应 odometry、local
  mapping、loop closure/global mapping（S013-S033）。
- 工作流包含 initialization、odometry、local mapping、loop closure 和
  global mapping 五个模块（S073-S080）。
- 数据金字塔把原始 scan、keyframe 和 submap 分层管理；论文示例中 10 个
  scan 合成一个 keyframe，10 个 keyframe 合成一个 submap（S080）。
- 局部建图在滑动窗口中联合优化近期状态和局部地图，窗口外状态边缘化后形成
  keyframe；这解释了为什么每帧直接累积点云不能等价于局部建图（S073-S080）。
- 实验同时报告 ATE、退化场景地图一致性、多会话回环后的 ATE、每模块耗时和
  内存（S268-S324）。

### 对 FusionLynx 的结论

1. `state_fusion_backend` 应继续只负责融合状态和输出 `/odom`，不能把
   `fusionlynx_mapping` 的点云累积误认为完整 SLAM。
2. 第一阶段接入 RTAB-Map 是合理的工程取舍，但它依赖输入轨迹的时间、TF
   和坐标语义正确；不能用错误的 `/odom` 去期待地图后端自动修复。
3. 后续若自研局部建图，应先加入滑动窗口/局部 scan matching，再考虑回环和
   全局优化；当前不应提前宣称具备这些能力。
4. 论文对退化和 z 轴一致性的讨论支持继续记录 LIO 健康状态、创新量、地图
   质量和轨迹误差，而不是只看 RViz 视觉效果。

## 2. Active SLAM Survey

来源：`02_Active_SLAM_Survey_TRO_2023.pdf`，MinerU：
`.mineru_output/selected_2026-08-23/active_slam_survey_2023/active_slam_survey_2023/auto/active_slam_survey_2023.md`

### 关键内容

- 综述把主动 SLAM 表述为定位、建图和运动决策的联合问题，并强调可复现
  benchmark 的必要性（S001-S025）。
- 工程上常见的模块化方案把候选动作识别、效用计算、动作选择/执行分成三阶段
  （S131-S143）。
- 地图表达包括拓扑、度量、度量-语义和混合地图；点云、occupancy grid、
  OctoMap 和 voxel map 都属于度量地图路线（S149-S159）。

### 对 FusionLynx 的结论

当前项目不是主动探索系统，因此不实现动作规划；但“模块职责解耦”和
“可复现实验”直接适用于 FusionLynx：融合、建图、评估和日志要有清晰接口，
不能让 RTAB-Map 反向成为 `/odom` 的发布者。

## 3. SuperVINS

来源：`10_SuperVINS_2024.pdf`，MinerU：
`.mineru_output/selected_2026-08-23/supervins/supervins/auto/supervins.md`

该论文是视觉-惯性方向，不能直接作为当前 LiDAR-轮速融合实现依据。它的
可迁移经验是：输入、前端、后端、回环和 ROS 数据传输应在系统图中分层，
滑动窗口和回环属于后端/长期关联职责（S058-S068）。实验同时报告 ATE、
旋转/平移 RPE，并使用多序列比较和轨迹可视化（S154-S170）。因此它只作为
未来多传感器扩展和实验报告结构参考，不改变当前 IMU/MAG health-only 契约。

## 4. Gaussian Splatting SLAM

来源：`07_Gaussian_Splatting_SLAM_CVPR2024.pdf`，MinerU：
`.mineru_output/selected_2026-08-23/gaussian_splatting_slam/gaussian_splatting_slam/auto/gaussian_splatting_slam.md`

该论文研究视觉稠密重建，不是当前 LiDAR 地图后端的直接替代方案。其关键
可迁移点是 keyframe 不是固定每帧保存，而是根据共视关系和相对平移选择；
并通过小窗口、插入/裁剪和正则化控制地图规模与质量（S127-S153）。实验
同时区分轨迹 ATE 和地图渲染质量指标（S155-S195）。这支持 FusionLynx
后续将“轨迹精度”和“地图质量”分开评估，但当前 RTAB-Map 仍采用点云地图
和已有的融合 `/odom`。

## 5. 归档边界

本批只归档 4 篇有明确用途的文献。`docs/paper` 中的重复论文、错误
arXiv 编号、纯语义/神经表示方向和与当前课题无直接关系的文件不计入融合
算法依据；Vault 的文献审计仍以正文核验为准，不能把候选文件数量写成已完成
的全文阅读数量。

## 6. 当前实验建议

根据上述论文，FusionLynx 下一轮实验至少记录：

- `/fast_lio/odom`、`/motioncore/wheel_odom`、`/odom` 的时间戳和频率；
- LIO 健康状态转换次数、持续时间、NIS、创新量和因子接纳率；
- ATE、平移/旋转 RPE、z 轴漂移、静止段位姿方差；
- RTAB-Map 的输入同步成功率、TF 查询失败次数、建图点数和处理耗时；
- 每个序列的硬件、参数、起止时间、是否回环和是否出现退化。
