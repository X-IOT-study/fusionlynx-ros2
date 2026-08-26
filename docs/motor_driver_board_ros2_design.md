# 电机驱动板 ROS 2 软件包设计文档

## 1. 设计目标

基于 `Motor-Driver-Board-Firmware` 设计一套 ROS 2 软件包，把 STM32 电机板抽象成标准机器人底盘驱动层，并能直接接入导航、EKF、IMU 过滤和遥操作。

核心目标：

1. 把固件的串口/CAN 协议封装成 ROS 2 节点。
2. 用标准话题替换旧工程里的 `/sub_vel`、`/pub_odom` 这类非标准命名。
3. 用生命周期节点管理硬件连接、失联恢复和安全停机。
4. 把描述、启动、控制、接口拆成独立包，便于维护和替换。

## 2. 从固件反推的能力边界

固件已经明确暴露了这些能力：

- 四轮麦克纳姆底盘控制。
- 编码器速度/里程数据。
- IMU 姿态/原始数据。
- 电池、电机 PID、偏航 PID、蜂鸣器、自动上报、时间同步、心跳。
- 数据帧格式固定为 `0xFF + DeviceID + L + Func + Data + Checksum`，最大帧长 69 字节。

关键约束：

- `Motion_Get_Encoder()` 说明编码器是底盘里程的基础。
- `config.h` 里的任务周期提示驱动侧要按 10 ms 到 30 ms 级别处理运动和 IMU 数据。
- `app_flash.*` 说明 `auto_report`、PID、测试模式等状态会持久化，ROS 2 侧要把“运行态”和“保存态”分开。
- `protocol_lib.h` 的功能码表已经足够支撑底盘控制，不需要先发明新协议。

## 3. 推荐的软件包拆分

### 3.1 必选包

**`motor_driver_board_driver`**

- 负责串口/CAN 通信、协议解析、命令下发、状态上报。
- 建议做成生命周期节点。
- 节点内部再拆一个纯库层，放协议编解码、串口收发、帧校验。

**`motor_driver_board_description`**

- 只放 URDF/xacro、关节命名、惯量、坐标系、RViz 资源。
- 关节名建议直接沿用旧工程：
  - `front_right_joint`
  - `front_left_joint`
  - `back_right_joint`
  - `back_left_joint`

**`motor_driver_board_bringup`**

- 放 launch、参数、EKF 配置、IMU 过滤配置、RViz 配置、遥操作启动脚本。
- 不写硬件逻辑。

### 3.2 可选包

**`motor_driver_board_interfaces`**

- 如果你希望把 PID、灯效、蜂鸣器、状态码做成强类型接口，再单独放 msg/srv。
- 如果只保留标准话题和少量服务，这个包可以先不建。

**`motor_driver_board_calibration`**

- 放 IMU 标定、线速度/角速度标定、底盘参数标定工具。
- 不要塞进驱动包。

**`motor_driver_board_ros2_control`**

- 仅在你准备接 `ros2_control` 时再加。
- 当前这块板的固件协议更适合先做 topic/service 驱动，再决定是否引入硬件接口插件。

## 4. 节点设计

### 4.1 主驱动节点

建议节点名：`motor_driver_board_node`

职责：

- 打开串口或 CAN。
- 上电后完成握手、时间同步、自动上报设置。
- 订阅 `/cmd_vel`，下发 `FUNC_MOTION`。
- 接收板端上报，转换成 ROS 2 标准消息。
- 监测心跳和超时，失联时立即停轮。

生命周期建议：

- `on_configure()`：读参数、打开协议对象、校验端口。
- `on_activate()`：建立连接、启动收发线程、下发自动上报和心跳。
- `on_deactivate()`：发送停车命令、关闭通信、停止后台线程。
- `on_error()`：进入安全态，清空队列，要求重新配置。

### 4.2 标准话题映射

| ROS 2 话题 | 消息类型 | 方向 | 备注 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 对应 `FUNC_MOTION` |
| `/odom` | `nav_msgs/msg/Odometry` | 发布 | 轮速里程计或融合前里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布 | 四个轮子 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 发布 | 原始或融合 IMU |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 发布 | 如固件侧提供磁场数据 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 发布 | 电压从 board 状态映射 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 发布 | 心跳、CRC、掉线、低电压 |

建议不要继续使用旧 ROS 1 风格的 `/sub_vel`、`/pub_odom`、`/pub_imu`，直接换成标准命名，再靠 launch remap 兼容旧上层。

### 4.3 服务设计

建议把“短请求、立刻反馈”的控制做成服务：

- `set_auto_report`
- `set_buzzer`
- `set_led_effect`
- `set_pid`
- `get_pid`
- `sync_time`
- `clear_yaw`
- `reset_board`（默认可禁用，避免误触）

这些服务比 topic 更适合 PID、灯效、蜂鸣器这类一次性指令。

## 5. 协议与数据映射

### 5.1 功能码分组

- 运动控制：`0x10`, `0x11`, `0x12`
- 上报数据：`0x0A`, `0x0B`, `0x0C`, `0x0E`
- PID：`0x13`, `0x14`, `0x15`, `0x16`
- 系统：`0x01`, `0x02`, `0x70`, `0x71`, `0x72`

### 5.2 建议的转换规则

- 轮速/编码器数据 -> `JointState` + `Odometry`
- IMU 原始数据 -> `Imu`
- 磁场数据 -> `MagneticField`
- 电池电压 -> `BatteryState`
- 心跳/CRC/掉线 -> `diagnostics`

### 5.3 坐标系与单位

- 里程计使用 `odom -> base_link` 或 `odom -> base_footprint`。
- `Imu.header.frame_id` 建议用 `imu_link`。
- 如果固件输出是磁北或 NED，需要在驱动侧转成 ROS 常用 ENU，再给 `robot_localization`。
- `JointState.position` 用弧度，`velocity` 用 rad/s。

## 6. 参数设计

建议至少支持这些参数：

- `port`
- `baud_rate`
- `transport_type` (`uart` / `can`)
- `frame_prefix`
- `wheel_names`
- `wheel_radius`
- `wheel_base_x`
- `wheel_base_y`
- `encoder_ppr`
- `publish_rate_hz`
- `heartbeat_timeout_ms`
- `cmd_vel_timeout_ms`
- `enable_auto_report`
- `enable_imu_filter`
- `enable_low_battery_alarm`
- `save_pid_to_flash`

旧工程里的 dynamic reconfigure 思路在 ROS 2 里应改成参数 + 服务，不要照搬。

## 7. 启动链路

建议的 bringup 顺序：

1. 驱动节点先启动并完成握手。
2. `robot_state_publisher` 发布 TF。
3. `imu_filter_madgwick` 可选启动，融合原始 IMU 和磁场。
4. `robot_localization` 启动 EKF，输入 `/odom` 和 `/imu/data`。
5. `rviz2` 或导航栈最后启动。

如果要复用旧的 `robot_localization.yaml`，就在 bringup 里把 `/odom` remap 到 `/odom0`，把 `/imu/data` remap 到 `/imu0`，这样可以直接沿用原来的 EKF 参数习惯。

launch 文件建议全部用 Python 写，便于参数条件和 remap。

## 8. 与旧源码的对应关系

| 旧源码 | 新 ROS 2 建议 |
|---|---|
| `yahboomcar_bringup` | `motor_driver_board_bringup` |
| `yahboomcar_description` | `motor_driver_board_description` |
| `yahboomcar_ctrl` | 拆成 `teleop` 或工具包，不放进驱动核心 |
| `yahboomcar_msgs` | 只保留真正需要的接口；不必整包迁移 |
| `base.cpp` | 不建议原样迁移，功能并入驱动节点或 EKF 后处理 |
| `Mcnamu_driver.py` | 可作为 ROS 2 Python 原型，但生产版建议转 C++ 生命周期节点 |

## 9. 关键风险

- 不能默认 IMU 坐标就是 ENU。
- 不能在回调里同步等服务，容易卡住执行器。
- 不能把板端专有协议直接暴露成一堆非标准 topic。
- 不能让驱动节点在掉线后继续保持 `/cmd_vel` 有效。
- 不能把 PID 保存、板卡复位这类危险操作默认开放给所有上层节点。

## 10. 推荐实施顺序

1. 先做协议库和串口收发层。
2. 再做生命周期驱动节点。
3. 再做 description 和 bringup。
4. 再接 EKF 和 IMU 过滤。
5. 最后补 calibration 工具和可选 ros2_control 路线。

## 11. 结论

这块板子的 ROS 2 方案，最稳妥的是“标准话题 + 生命周期驱动 + bringup 分层 + 可选接口包”。  
如果后续要进一步标准化，再考虑把底盘控制拆成 `ros2_control` 硬件插件。
