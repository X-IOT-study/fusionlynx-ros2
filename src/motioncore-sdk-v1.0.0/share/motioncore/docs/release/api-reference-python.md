# MotionCore Python API Reference

This document describes the public Python package surface implemented in the current release.

## Data models

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

- `value` (nanoseconds)
- `sec`
- `nsec`
- `from_sec_nsec(...)`
- `to_sec_nsec()`

## `motioncore.MeasurementHeader`

- `timestamp` (nanoseconds)
- `sequence`
- `flags`
- `timestamp_sec`
- `timestamp_nsec`

## `motioncore.Client`

Implemented methods:

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

## Notes

- `set_motion()` builds the `FUNC_MOTION (0x12)` payload.
- `send_motion()` writes that payload to `0xFC/0x12` on the configured transport.
- Python `send_time_sync()` / `send_heartbeat()` build payload bytes; send them with `write_frame()`.
- `MotionCommand` should be constructed with named arguments across languages; the declaration order differs from C++.
- The firmware timestamp source is millisecond-resolution, so telemetry timestamps are quantized to 1 ms.
