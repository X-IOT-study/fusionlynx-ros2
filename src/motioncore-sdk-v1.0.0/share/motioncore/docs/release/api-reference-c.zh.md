# MotionCore C API 参考

本文档描述当前 release 里已经实现的 C 公共接口。

## 协议核心

- `mc_checksum_compute(const mc_u8 *bytes, size_t length)`
- `mc_frame_encode(...)`
- `mc_frame_validate(...)`
- `mc_frame_decode(...)`

## 遥测编码/解码

- `mc_decode_speed_report(...)`
- `mc_decode_imu_attitude(...)`
- `mc_decode_raw_imu(...)`
- `mc_decode_encoder_counts(...)`
- `mc_decode_status_report(...)`
- `mc_decode_pid_report(...)`
- `mc_encode_speed_report(...)`
- `mc_encode_pid_report(...)`

## 负载辅助

- `mc_encode_request_pid_payload(...)`
- `mc_encode_motion_command(...)`
- `mc_encode_time_sync_payload(...)`
- `mc_encode_heartbeat_payload(...)`

## 传输与会话

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

## 时间戳说明

`mc_encode_time_sync_payload()` 和测量头都使用 ROS 风格的 `sec + nsec` 线格式，但当前固件测量时间源来自 `Port_GetTickMs()`，所以遥测时间戳的实际分辨率是 **1 ms**。

`mc_encode_motion_command()` 会打包固件 `FUNC_MOTION (0x12)` 解析的 7 字节 payload：第 1 字节是原始控制字节，后面依次是 `vx`、`vy`、`vz` 的有符号 16 位小端编码。
