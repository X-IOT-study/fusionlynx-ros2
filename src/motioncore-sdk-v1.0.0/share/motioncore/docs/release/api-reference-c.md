# MotionCore C API Reference

This document describes the public C SDK surface implemented in the current release.

## Core protocol helpers

- `mc_checksum_compute(const mc_u8 *bytes, size_t length)`
- `mc_frame_encode(mc_u8 device_id, mc_u8 function_code, const mc_u8 *payload, size_t payload_len, mc_u8 *out, size_t out_len)`
- `mc_frame_validate(const mc_u8 *frame, size_t frame_len)`
- `mc_frame_decode(const mc_u8 *frame, size_t frame_len, mc_frame_t *out_frame)`

## Telemetry codecs

- `mc_decode_speed_report(...)`
- `mc_decode_imu_attitude(...)`
- `mc_decode_raw_imu(...)`
- `mc_decode_encoder_counts(...)`
- `mc_decode_status_report(...)`
- `mc_decode_pid_report(...)`
- `mc_encode_speed_report(...)`
- `mc_encode_pid_report(...)`

## Payload helpers

- `mc_encode_request_pid_payload(...)`
- `mc_encode_motion_command(...)`
- `mc_encode_time_sync_payload(...)`
- `mc_encode_heartbeat_payload(...)`

## Transport and session

- `mc_transport_init(...)`
- `mc_transport_read(...)`
- `mc_transport_write(...)`
- `mc_transport_close(...)`
- `mc_client_init(...)`
- `mc_client_set_transport(...)`
- `mc_client_set_handlers(...)`
- `mc_client_send_frame(...)`
- `mc_client_poll(...)`
- `mc_client_feed_bytes(...)`

## Timestamp note

`mc_encode_time_sync_payload()` and the measurement header use ROS-style `sec + nsec` on wire, but current firmware measurement timestamps come from `Port_GetTickMs()`. In practice that means telemetry timestamps are quantized to **1 ms**.

`mc_encode_motion_command()` packs the 7-byte `FUNC_MOTION (0x12)` payload consumed by the firmware parser: a raw control byte, then `vx`, `vy`, and `vz` as signed 16-bit little-endian values.
