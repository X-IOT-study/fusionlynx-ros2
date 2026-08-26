# MotionCore Python API 参考

本文档描述当前 release 里已经实现的 Python 包接口。

## 数据模型

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

## `motioncore.Timestamp`

- `value`（纳秒）
- `sec`
- `nsec`
- `from_sec_nsec(...)`
- `to_sec_nsec()`

## `motioncore.MeasurementHeader`

- `timestamp`（纳秒）
- `sequence`
- `flags`
- `timestamp_sec`
- `timestamp_nsec`

## `motioncore.Client`

已实现方法：

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
- `send_motion()`
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

## `motioncore.TransportSerial`

- `open()`
- `close()`
- `is_open`
- `write()`
- `read()`

## 备注

- `set_motion()` 负责组装 `FUNC_MOTION (0x12)` 的 payload。
- `send_motion()` 会把这个 payload 直接写到已配置的传输层，目标帧是 `0xFC/0x12`。
- Python `send_time_sync()` / `send_heartbeat()` 返回的是 payload，需要配合 `write_frame()` 发送。
- 跨语言构造 `MotionCommand` 时请使用命名字段，不要依赖位置参数。
- 固件测量时间源是毫秒 tick，所以遥测时间戳实际分辨率是 1 ms。
