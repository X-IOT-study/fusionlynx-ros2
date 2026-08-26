---
created: 2026-08-23
updated: 2026-08-23
owner: 徐文圣
status: active
tags: [decision, state-fusion, lio, logging]
---

# State Fusion LIO 状态日志决策

## 背景

当前系统的主要问题不是缺少普通进程日志，而是无法解释 FAST-LIO 在
何时被降权、拒绝、超时或重新接纳。为避免逐帧输出影响实时性，日志按
状态转换记录，逐帧数值继续放在 `/motioncore/diagnostics`。

日志字段和等级参考 `/home/dream/Documents/work_xxxx/for_slam` 中的
SLAM 日志实践材料；该目录不作为论文来源。

## 实施

`state_fusion_backend_node` 新增 LIO 状态转换日志：

- 首次状态记录为 `UNINITIALIZED -> <state>`；
- `HEALTHY` 使用 `INFO`；
- `DEGRADED`、`RECOVERING` 使用 `WARN`；
- `FAILED` 使用 `ERROR`；
- 同一状态连续出现时不重复打印；
- LIO 超时由固定频率定时器根据 steady-clock 接收间隔触发
  `check_timeout()` 后记录；不再比较 FAST-LIO 与 Wheel 的消息时间戳。

每次转换记录 ROS 时间戳、前后状态、原因、因子是否接纳、是否需要对齐、
静止检测、协方差缩放、NIS、创新范数和 Wheel/IMU/MAG 一致性。超时额外
记录节点接收年龄、最新消息时间戳和节点当前时间，避免把消息时间误认为
接收时间。

## 验证

```text
CCACHE_DISABLE=1 colcon build --packages-select state_fusion_backend
ctest --test-dir build/state_fusion_backend --output-on-failure
```

结果：构建成功，`test_lio_health_monitor` 和 `test_fusion_backend` 均通过。

## 约束

本改动不新增 TF 发布者，不改变 Wheel/LIO/IMU 话题和时间同步合同，不改变
IMU/MAG 的 health-only 角色，也不让日志成为新的高频数据通道。
