# MotionCore C++ API 参考

本文档描述当前 release 里已经实现的 C++ 包装层接口。

## 数据类型

- `motioncore::Timestamp`
- `motioncore::MeasurementHeader`
- `motioncore::Frame`
- `motioncore::Transport`
- `motioncore::SpeedReport`
- `motioncore::ImuAttitude`
- `motioncore::RawImuSample`
- `motioncore::EncoderCounts`
- `motioncore::StatusReport`
- `motioncore::PidReport`
- `motioncore::PidCommand`
- `motioncore::MotionCommand`

## `motioncore::Timestamp`

- `value`
- `sec()`
- `nsec()`
- `fromSecNsec(...)`

## `motioncore::MeasurementHeader`

- `timestamp`
- `sequence`
- `flags`
- `timestampSec()`
- `timestampNsec()`

## `motioncore::Frame`

- `Frame::encode(...)`
- `Frame::decode(...)`
- `Frame::native()`

## `motioncore::Transport`

- `Transport::bind(...)`
- `Transport::native()`

## `motioncore::Client`

- `setTransport(...)`
- `setHandlers(...)`
- `sendFrame(...)`
- `poll()`
- `feedBytes(...)`
- `sendTimeSync(...)`
- `sendHeartbeat()`
- `requestData(...)`
- `setMotorPid(...)`
- `setYawPid(...)`
- `setMotion(...)`
- `resetState()`
- `resetFlash()`
- `setAutoReport(...)`
- `setBeep(...)`
- `testMode(...)`
- `clearYaw()`

## 备注

- 当前包装层 **没有** `setMotor()` 和 `setCarRun()`。
- `setMotion()` 会直接发送固件 `FUNC_MOTION (0x12)` 帧；`MotionCommand.yaw_adjust` 是原始控制字节。
- `sendTimeSync()` 接收的是纳秒时间戳。
- 固件测量时间来自毫秒 tick，所以 wire 上的时间戳实际分辨率是 1 ms。
