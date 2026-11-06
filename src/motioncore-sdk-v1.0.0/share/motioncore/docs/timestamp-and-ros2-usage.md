# MotionCore SDK 时间戳与 ROS 2 使用指南

本文按当前代码实现编写，不按设计草案写法。凡是和 `docs/design/*` 冲突的地方，以这里和 `docs/release/*` 为准。

## 1. 先说结论

- SDK 里的 `Timestamp` 和 `MeasurementHeader.timestamp` 都按 **纳秒** 表达。
- 固件当前真正的时间源是 `Port_GetTickMs()`，所以测量时间戳的实际分辨率是 **1 ms**。
- 设备侧没有把 encoder / IMU / status 自动对齐到同一时刻，不能假定它们的 `timestamp_ns` 完全相同。
- `send_time_sync()` 是同步握手/锚点辅助，不是把测量时间改成主机时间的开关。
- `set_motion()` 负责组装 `FUNC_MOTION (0x12)` 的 payload，`send_motion()` 负责直接发帧。
- 当前固件没有把 `PROTOCOL_MEAS_FLAG_SYNCED` 作为测量包的常规输出标志。

## 2. 当前可用的 SDK 面

### C core

当前实现的核心函数包括：

- `mc_frame_encode()`
- `mc_frame_decode()`
- `mc_frame_validate()`
- `mc_checksum_compute()`
- `mc_decode_speed_report()`
- `mc_decode_imu_attitude()`
- `mc_decode_raw_imu()`
- `mc_decode_encoder_counts()`
- `mc_decode_status_report()`
- `mc_decode_pid_report()`
- `mc_encode_speed_report()`
- `mc_encode_pid_report()`
- `mc_encode_request_pid_payload()`
- `mc_encode_motion_command()`
- `mc_encode_time_sync_payload()`
- `mc_encode_heartbeat_payload()`
- `mc_client_*()`
- `mc_transport_*()`

### C++ wrapper

当前公开类型和方法包括：

- `motioncore::Timestamp`
- `motioncore::MeasurementHeader`
- `motioncore::Frame`
- `motioncore::Transport`
- `motioncore::Client`

`Client` 的实际方法是：

- `setTransport()`
- `setHandlers()`
- `sendFrame()`
- `poll()`
- `feedBytes()`
- `sendTimeSync()`
- `sendHeartbeat()`
- `requestData()`
- `setMotorPid()`
- `setYawPid()`
- `setMotion()`
- `resetState()`
- `resetFlash()`
- `setAutoReport()`
- `setBeep()`
- `testMode()`
- `clearYaw()`

### Python wrapper

当前公开类型和方法包括：

- `motioncore.Timestamp`
- `motioncore.MeasurementHeader`
- `motioncore.SpeedReport`
- `motioncore.ImuAttitude`
- `motioncore.RawImuSample`
- `motioncore.EncoderCounts`
- `motioncore.StatusReport`
- `motioncore.PidReport`
- `motioncore.PidCommand`
- `motioncore.MotionCommand`
- `motioncore.Client`
- `motioncore.TransportSerial`

Python `Client` 的实际方法是：

- `set_transport()`
- `send_frame()`
- `write_frame()`
- `send_time_sync()`
- `set_measurement_handlers()`
- `set_handlers()`
- `connect()`
- `disconnect()`
- `feed_bytes()`
- `poll()`
- `send_heartbeat()`
- `request_data()`
- `request_motor_pid()`
- `request_yaw_pid()`
- `set_motor_pid()`
- `set_yaw_pid()`
- `set_motion()`
- `reset_state()`
- `reset_flash()`
- `set_auto_report()`
- `set_beep()`
- `test_mode()`
- `clear_yaw()`
- `decode_speed()`
- `decode_imu()`
- `decode_raw_imu()`
- `decode_encoder()`
- `decode_status()`

## 3. 时间戳到底怎么用

### 3.1 设备侧语义

`APP/Src/app.c` 里的 `App_Get_Device_Timestamp()` 返回的是设备本地 tick。当前实现来自 `Port_GetTickMs()`，所以它是一个**单调设备时间**，但不是主机时间，也不是高精度硬件时钟。

这意味着：

- 时间戳在逻辑上是“设备上第几毫秒”
- 在 wire 上被编码成 ROS 风格的 `sec + nsec`
- `nsec` 部分会是 `1_000_000` 的整数倍

### 3.2 SDK 侧语义

SDK 把时间戳统一抽象成纳秒：

- Python: `Timestamp.value`
- C++: `motioncore::Timestamp::value`
- 头部: `MeasurementHeader.timestamp`

辅助方法：

- Python: `Timestamp.from_sec_nsec()` / `to_sec_nsec()`
- Python: `MeasurementHeader.timestamp_sec` / `timestamp_nsec`
- C++: `Timestamp::fromSecNsec()` / `sec()` / `nsec()`
- C++: `MeasurementHeader::timestampSec()` / `timestampNsec()`

### 3.3 `send_time_sync()` 的真实作用

`send_time_sync()` / `sendTimeSync()` 发送的是**主机侧时间锚点**，不是对设备测量时间的重写。

它适合做这几件事：

- 建立主机时间和设备时间的锚点
- 检查链路是否正常
- 做日志对齐

它不适合做这几件事：

- 期待 encoder / IMU / status 自动同戳
- 期待设备测量时间变成 ROS 系统时间

## 4. 怎么在 SDK 里取时间戳

### Python

```python
import time
from motioncore import Client, Timestamp, TransportSerial

client = Client()
transport = TransportSerial(port="/dev/ttyUSB0", baudrate=115200)
transport.open()
client.set_transport(transport)

stamp = Timestamp(time.time_ns())
payload = client.send_time_sync(stamp)
client.write_frame(0xFC, 0x70, payload)
```

如果你只想处理固件回来的数据：

```python
def on_speed(msg):
    print(msg.header.timestamp, msg.header.timestamp_sec, msg.header.timestamp_nsec)

client.set_measurement_handlers(on_speed=on_speed)
while True:
    client.poll()
```

### C++

```cpp
#include <chrono>

motioncore::Client client;
const auto now_ns = static_cast<mc_u64>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
client.sendTimeSync(now_ns);
```

如果你拿到的是回调里的 `SpeedReport` / `EncoderCounts` / `StatusReport`，直接读 `header.timestamp` 即可。

### C core

低层场景直接用：

```c
uint8_t payload[8];
mc_encode_time_sync_payload(payload, sizeof(payload), timestamp_ns);
```

## 5. ROS 2 中怎么融合

### 5.1 发布原则

ROS 2 里最稳妥的做法是：

- 用设备时间作为测量时间线
- 在桥接节点里把设备时间映射到 ROS 时间
- 把原始设备时间另存为自定义字段或日志字段

如果你要发布标准消息：

- `header.stamp` 填 ROS 时间
- `header.frame_id` 填传感器/底盘坐标系
- 不要用序号代替时间戳

### 5.2 锚点映射

推荐方式是“设备锚点 + ROS 锚点”：

```text
ros_stamp = ros_anchor + (device_stamp - device_anchor)
```

首次收到数据或首次完成 `send_time_sync()` 后，记录：

- `device_anchor`
- `ros_anchor = node.get_clock().now()`

后续每条消息都按这个差值转成 ROS `stamp`。

### 5.3 融合策略

不要用“完全同一纳秒”去配 encoder 和 IMU。它们来自不同任务，当前固件分辨率又只有 1 ms。

优先用：

- `message_filters::ApproximateTime`
- 设备时间窗口匹配
- 必要时做插值/重采样

如果你接 `robot_localization`，用的是：

- 带 `header.stamp` 的 topic
- 合理的 `SensorDataQoS`
- 合理的 `sensor_timeout`

### 5.4 QoS 建议

- 高速传感器流：`BEST_EFFORT` / `SensorDataQoS()`
- 命令与同步：`RELIABLE`、`depth = 1`
- 状态/诊断：`RELIABLE`、`depth = 10`

## 6. `demo.csv` 为什么不能直接当同步表

`demo-test/live_serial_demo.py` 现在导出的 CSV 是**长表**，不是“所有传感器共用一行”的宽表。

每行都有：

- `sample_type`
- `timestamp_ns`
- `timestamp_sec`
- `timestamp_nsec`
- `sequence`
- `flags`

这代表：

- 一行就是一个样本
- 不同样本类型可能落在同一个 `timestamp_ns`
- 也可能相差 1 到几个 tick

所以正确处理方式是：

- 按 `sample_type` 分组
- 按时间窗对齐
- 不要用字符串表格直接做严格等值 join

## 7. API 文档和代码一致性检查

### 一致的地方

- `Timestamp` / `MeasurementHeader` 的时间语义一致
- `request_data()` / `requestData()` 都存在
- `set_motion()` / `setMotion()` 都存在
- `reset_state()` / `resetState()` 都存在
- `TransportSerial` 和 `feed_bytes()` / `poll()` 的使用方式一致

### 需要注意的地方

- Python `send_time_sync()` / `send_heartbeat()` 返回的是**payload**，要再配合 `write_frame()` 发送
- C++ `sendTimeSync()` / `sendHeartbeat()` 是直接发送
- Python `send_time_sync()` 接收 `Timestamp`（其 `.value` 为纳秒），C++ `sendTimeSync()` 接收纳秒整数
- `motioncore::MotionCommand` 和 Python `MotionCommand` 的字段声明顺序不同，跨语言请用**命名字段**，不要依赖位置参数
- `setMotor()` / `setCarRun()` 不是当前 release 里的实现，不应该再当作现成 API 使用
- `mc_client_connect()` / `mc_client_disconnect()`、`mc_decode_heartbeat_response()`、`mc_decode_time_sync_response()` 不是当前 C core 的实现

### 结论

当前 release 的真实接口，应该以 `cpp/include/motioncore/*.hpp`、`include/motioncore/*.h`、`python/motioncore/*.py` 和 `docs/release/*` 为准。设计文档可以保留更远期的规划，但不要把规划项当成已实现 API。
