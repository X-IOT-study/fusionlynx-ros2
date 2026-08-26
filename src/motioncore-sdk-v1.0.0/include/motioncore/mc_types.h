/**
 * @file            mc_types.h
 * @brief           MotionCore shared type definitions header
 *                  MotionCore 共享类型定义头文件
 * @version         1.0
 * @description     Declares the scalar aliases and telemetry payload
 *                  structures shared across the C API.
 *                  声明 C API 之间共享的标量别名和遥测载荷结构体。
 *
 * @type_list
 * 1. mc_u8 - Unsigned protocol byte / 无符号协议字节
 * 2. mc_u16 - Unsigned 16-bit integer / 无符号 16 位整数
 * 3. mc_u32 - Unsigned 32-bit integer / 无符号 32 位整数
 * 4. mc_i16 - Signed 16-bit integer / 有符号 16 位整数
 * 5. mc_i32 - Signed 32-bit integer / 有符号 32 位整数
 * 6. mc_u64 - Unsigned 64-bit integer / 无符号 64 位整数
 * 7. mc_measurement_header_t - Common telemetry header / 公共遥测头
 * 8. mc_speed_report_t - Speed telemetry payload / 速度遥测载荷
 * 9. mc_pid_command_t - PID command payload / PID 控制载荷
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#ifndef MOTIONCORE_MC_TYPES_H
#define MOTIONCORE_MC_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol byte type used throughout the MotionCore C API. */
typedef uint8_t mc_u8;
/** Unsigned 16-bit integer used by the MotionCore C API. */
typedef uint16_t mc_u16;
/** Unsigned 32-bit integer used by the MotionCore C API. */
typedef uint32_t mc_u32;
/** Signed 16-bit integer used by the MotionCore C API. */
typedef int16_t mc_i16;
/** Signed 32-bit integer used by the MotionCore C API. */
typedef int32_t mc_i32;
/** Unsigned 64-bit integer used by the MotionCore C API. */
typedef uint64_t mc_u64;

/** Measurement header flags published by the firmware. */
enum {
    MC_MEASUREMENT_FLAG_VALID = 0x01u,
    MC_MEASUREMENT_FLAG_SYNCED = 0x02u,
    MC_MEASUREMENT_FLAG_DERIVED = 0x04u,
    MC_MEASUREMENT_FLAG_FILTERED = 0x08u,
    MC_MEASUREMENT_FLAG_LATCHED = 0x10u,
};

/**
 * Common measurement header used by telemetry frames.
 *
 * The firmware encodes the header as timestamp + sequence + flags.
 */
typedef struct {
    mc_u64 timestamp;
    mc_u16 sequence;
    mc_u8 flags;
} mc_measurement_header_t;

/**
 * Timestamp wrapper used by the SDK to keep time values explicit.
 */
typedef struct {
    mc_u64 value;
} mc_timestamp_t;

/** Speed report payload decoded from function code 0x0A. */
typedef struct {
    mc_measurement_header_t header;
    mc_i16 vx;
    mc_i16 vy;
    mc_i16 vz;
} mc_speed_report_t;

/** IMU attitude payload decoded from function code 0x0B. */
typedef struct {
    mc_measurement_header_t header;
    mc_i16 roll;
    mc_i16 pitch;
    mc_i16 yaw;
} mc_imu_attitude_t;

/** Raw IMU payload decoded from function code 0x0C. */
typedef struct {
    mc_measurement_header_t header;
    mc_i16 gyro_x;
    mc_i16 gyro_y;
    mc_i16 gyro_z;
    mc_i16 accel_x;
    mc_i16 accel_y;
    mc_i16 accel_z;
    mc_i16 mag_x;
    mc_i16 mag_y;
    mc_i16 mag_z;
} mc_raw_imu_t;

/** Encoder counts payload decoded from function code 0x0E. */
typedef struct {
    mc_measurement_header_t header;
    mc_i32 encoder_1;
    mc_i32 encoder_2;
    mc_i32 encoder_3;
    mc_i32 encoder_4;
} mc_encoder_counts_t;

/** Status report payload decoded from function code 0x0F. */
typedef struct {
    mc_measurement_header_t header;
    mc_u8 fault_flags;
    mc_u16 battery_voltage;
    mc_u8 battery_state;
    mc_u8 system_enable;
    mc_u8 link_health;
} mc_status_report_t;

/** PID report payload decoded from function codes 0x15 and 0x16. */
typedef struct {
    mc_u8 index;
    mc_i16 kp;
    mc_i16 ki;
    mc_i16 kd;
} mc_pid_report_t;

/** PID command payload used when encoding set/request flows. */
typedef struct {
    mc_u8 index;
    mc_i16 kp;
    mc_i16 ki;
    mc_i16 kd;
    mc_u8 persist;
} mc_pid_command_t;

#ifdef __cplusplus
}
#endif

#endif
