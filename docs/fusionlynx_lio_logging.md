# FusionLynx LIO 日志规范

## 目的

日志用于回答“当前 LIO 为什么被接纳、降权、对齐或拒绝”，而不是只在进程崩溃时输出错误。日志实践参考
`/home/dream/Documents/work_xxxx/for_slam/SLAM工程实战：如何正确打日志解决90%漂移与玄学bug.html`：

- 记录算法状态和决策，不只记录异常；
- 每条关键记录包含时间、模块、状态、原因和量化指标；
- 显式记录传感器时间顺序和超时；
- 重要状态变化打印一次，连续帧指标放入诊断通道；
- `WARN` 表示可恢复退化，`ERROR` 表示传感器失效或不可恢复失败。

该 HTML 只作为日志工程实践来源，不是论文来源。

## 当前状态机

`LioHealthMonitor` 的状态为：

```text
HEALTHY -> DEGRADED -> FAILED
    ^          |          |
    |          v          v
    +------ RECOVERING <-+
```

实际状态转换由 `observe()` 和 `check_timeout()` 产生。`handle_lio()` 负责
记录有时间戳的 FAST-LIO 观测及其节点接收时刻，定时器负责用 steady clock
检测 LIO 接收间隔。超时判断不再使用 FAST-LIO 与 Wheel 的消息时间戳差值，
避免不同传感器时间域或传输延迟造成伪超时。

状态图只表示主要路径。`check_timeout()` 和 `observe()` 中的无效时间间隔可
以从任何状态直接进入 `FAILED`；`RECOVERING` 也可能因新的降权指标回到
`DEGRADED`，或因连续异常再次进入 `FAILED`。LIO 健康评估仅在
`use_lio_health: true` 时运行（Vault profile E–G）；默认 profile D 和
profile A–C 不产生这些状态转换日志。

## 等级约定

| 等级 | 触发条件 | 含义 |
|---|---|---|
| `INFO` | 首次初始化为 `HEALTHY`，或恢复为 `HEALTHY` | LIO 当前可正常参与因子图 |
| `WARN` | `DEGRADED` 或 `RECOVERING` | LIO 仍可能参与，但已增大协方差、需要对齐或暂不正常使用 |
| `ERROR` | `FAILED` | LIO 超时或不可恢复失效，后端进入轮速预测/回退路径 |

相同状态不会重复打印。高频指标由 `/motioncore/diagnostics` 提供，避免
日志 I/O 改变实时调度和 Wheel/LIO 时间同步。

## 状态转换日志字段

当前日志格式包含：

```text
timestamp
previous_state -> current_state
reason
accept_factor
requires_alignment
static
covariance_scale
nis
innovation_norm
wheel_consistency
imu_consistency
mag_consistency
```

watchdog 超时记录还包含 `receive_age`、`sensor_timeout`、最新 LIO 消息
时间戳和节点当前时间；这些字段只由 `check_timeout()` 路径产生。`observe()`
另外会拒绝相邻 FAST-LIO 消息头时间间隔 `dt <= 0` 或 `dt > sensor_timeout`
的观测，并以 `reason=stale or invalid interval` 直接进入 `FAILED`；该路径
只输出标准状态转换字段。

这些字段应能够直接对应健康监视器的判断和因子图决策。特别是：

- `nis` 和 `innovation_norm` 用于判断 LIO 与当前融合状态的创新是否异常；
- `accept_factor=false` 用于说明当前 LIO 没有进入图，而不是误认为输出停止；
- `requires_alignment=true` 用于识别恢复阶段的坐标对齐；
- `covariance_scale` 用于解释 LIO 仍被使用但对结果影响减弱；
- `static=true` 用于区分机器人静止期间的约束策略；
- `reason` 用于记录超时、跳变、传感器不一致等离散判断。

## 运行时查看

```bash
ros2 topic echo /motioncore/diagnostics
ros2 run rqt_console rqt_console
```

需要临时提高或降低 ROS 日志等级时使用 ROS 2 的日志参数或环境配置，
不通过修改算法阈值来实现。发布版本保留 `WARN`/`ERROR` 和简洁的状态
`INFO`，不打开逐帧 `DEBUG`。
