# MotionCore C++ API Reference

This document describes the public C++ wrapper surface implemented in the current release.

## Data types

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

## Notes

- `setMotor()` and `setCarRun()` are not implemented in the current wrapper.
- `setMotion()` sends the firmware `FUNC_MOTION (0x12)` frame directly; `MotionCommand.yaw_adjust` is the raw control byte.
- `sendTimeSync()` takes a nanosecond timestamp.
- The firmware currently sources telemetry timestamps from a millisecond tick, so wire timestamps are quantized to 1 ms.
