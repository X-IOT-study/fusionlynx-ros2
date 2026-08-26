/**
 * @file            mc_protocol.h
 * @brief           MotionCore protocol codec header
 *                  MotionCore 协议编解码头文件
 * @version         1.0
 * @description     Declares the payload codec helpers and protocol payload
 *                  builders used by the SDK and firmware-facing tooling.
 *                  声明 SDK 与固件侧工具使用的载荷编解码辅助函数和协议载荷构建函数。
 *
 * @function_list
 * 1. mc_decode_speed_report - Decode speed telemetry / 解码速度遥测
 * 2. mc_decode_imu_attitude - Decode IMU attitude telemetry / 解码姿态遥测
 * 3. mc_decode_raw_imu - Decode raw IMU telemetry / 解码原始 IMU 遥测
 * 4. mc_decode_encoder_counts - Decode encoder telemetry / 解码编码器遥测
 * 5. mc_decode_status_report - Decode status telemetry / 解码状态遥测
 * 6. mc_decode_pid_report - Decode PID telemetry / 解码 PID 遥测
 * 7. mc_encode_speed_report - Encode speed telemetry / 编码速度遥测
 * 8. mc_encode_pid_report - Encode PID telemetry / 编码 PID 遥测
 * 9. mc_encode_request_pid_payload - Encode PID request payload / 编码 PID 请求载荷
 * 10. mc_encode_time_sync_payload - Encode time sync payload / 编码时间同步载荷
 * 11. mc_encode_heartbeat_payload - Encode heartbeat payload / 编码心跳载荷
 * 12. mc_encode_motion_command - Encode motion command payload / 编码运动控制载荷
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#ifndef MOTIONCORE_MC_PROTOCOL_H
#define MOTIONCORE_MC_PROTOCOL_H

#include <motioncore/mc_frame.h>
#include <motioncore/mc_types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MC_FUNC_REQUEST_DATA 0x50u
#define MC_FUNC_REPORT_MOTOR_PID 0x15u
#define MC_FUNC_REPORT_YAW_PID 0x16u
#define MC_FUNC_TIME_SYNC 0x70u
#define MC_FUNC_HEARTBEAT_REQUEST 0x71u
#define MC_FUNC_HEARTBEAT_RESPONSE 0x72u

/**
 * Decode a speed report payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_speed_report(const mc_u8 *payload, size_t payload_len, mc_speed_report_t *out_report);

/**
 * Decode an attitude report payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_imu_attitude(const mc_u8 *payload, size_t payload_len, mc_imu_attitude_t *out_report);

/**
 * Decode a raw IMU payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_raw_imu(const mc_u8 *payload, size_t payload_len, mc_raw_imu_t *out_report);

/**
 * Decode an encoder counts payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_encoder_counts(const mc_u8 *payload, size_t payload_len, mc_encoder_counts_t *out_report);

/**
 * Decode a status payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_status_report(const mc_u8 *payload, size_t payload_len, mc_status_report_t *out_report);

/**
 * Decode a PID report payload.
 *
 * @param payload Raw payload bytes.
 * @param payload_len Payload length in bytes.
 * @param out_report Destination report.
 * @return 0 on success, negative value on failure.
 */
int mc_decode_pid_report(const mc_u8 *payload, size_t payload_len, mc_pid_report_t *out_report);

/**
 * Encode a speed report payload.
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @param report Report to encode.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_speed_report(mc_u8 *payload, size_t payload_len, const mc_speed_report_t *report);

/**
 * Encode a PID report payload.
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @param report Report to encode.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_pid_report(mc_u8 *payload, size_t payload_len, const mc_pid_report_t *report);

/**
 * Encode a request-data payload for a PID report request.
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @param request_function_code Request target function code.
 * @param request_index Optional target index; use 5 for yaw PID.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_request_pid_payload(mc_u8 *payload, size_t payload_len, mc_u8 request_function_code, mc_u8 request_index);

/**
 * Encode a time-sync payload.
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @param timestamp Host timestamp in nanoseconds to transmit.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_time_sync_payload(mc_u8 *payload, size_t payload_len, mc_u64 timestamp);

/**
 * Encode a heartbeat payload.
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_heartbeat_payload(mc_u8 *payload, size_t payload_len);

/**
 * Encode a motion command payload for FUNC_MOTION (0x12).
 *
 * Payload layout:
 * - byte 0: raw control byte (bit 7 enables yaw adjust)
 * - bytes 1-2: vx (int16 little-endian)
 * - bytes 3-4: vy (int16 little-endian)
 * - bytes 5-6: vz (int16 little-endian)
 *
 * @param payload Output buffer.
 * @param payload_len Output buffer size in bytes.
 * @param vx Linear velocity on X.
 * @param vy Linear velocity on Y.
 * @param vz Angular or rotational velocity.
 * @param yaw_adjust Raw control byte written to the first payload byte.
 * @return Encoded payload length on success, or 0 on failure.
 */
size_t mc_encode_motion_command(mc_u8 *payload, size_t payload_len, mc_i16 vx, mc_i16 vy, mc_i16 vz, mc_u8 yaw_adjust);

#ifdef __cplusplus
}
#endif

#endif
