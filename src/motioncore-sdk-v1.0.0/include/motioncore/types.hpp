/**
 * @file            types.hpp
 * @brief           MotionCore C++ shared types header
 *                  MotionCore C++ 共享类型头文件
 * @version         1.0
 * @description     Declares the C++ aliases and telemetry data models layered
 *                  on top of the MotionCore C API types.
 *                  声明建立在 MotionCore C API 类型之上的 C++ 别名和遥测数据模型。
 *
 * @type_list
 * 1. U8/U16/U32/I16/I32/U64 - Scalar aliases / 标量别名
 * 2. Timestamp - Nanosecond timestamp wrapper / 纳秒时间戳包装
 * 3. MeasurementHeader - Shared telemetry header / 共享遥测头
 * 4. SpeedReport - Speed telemetry model / 速度遥测模型
 * 5. ImuAttitude - IMU attitude telemetry model / IMU 姿态遥测模型
 * 6. RawImuSample - Raw IMU telemetry model / 原始 IMU 遥测模型
 * 7. EncoderCounts - Encoder telemetry model / 编码器遥测模型
 * 8. StatusReport - Status telemetry model / 状态遥测模型
 * 9. PidReport - PID telemetry model / PID 遥测模型
 * 10. PidCommand - PID command model / PID 控制模型
 * 11. MotionCommand - Motion control model / 运动控制模型
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#pragma once

#include <motioncore/mc_types.h>

namespace motioncore {

using U8 = mc_u8;
using U16 = mc_u16;
using U32 = mc_u32;
using I16 = mc_i16;
using I32 = mc_i32;
using U64 = mc_u64;

/** Number of nanoseconds per second. */
inline constexpr U64 NS_PER_SEC = 1000000000ULL;

/** Nanosecond timestamp wrapper used by the C++ API. */
struct Timestamp {
    /** Raw time value in nanoseconds. */
    U64 value{};

    /** Return the whole seconds part of the timestamp. */
    constexpr U32 sec() const noexcept {
        return static_cast<U32>(value / NS_PER_SEC);
    }

    /** Return the nanoseconds part within the current second. */
    constexpr U32 nsec() const noexcept {
        return static_cast<U32>(value % NS_PER_SEC);
    }

    /** Build a timestamp from seconds and nanoseconds. */
    static constexpr Timestamp fromSecNsec(U32 sec, U32 nsec) noexcept {
        return Timestamp{static_cast<U64>(sec) * NS_PER_SEC + static_cast<U64>(nsec)};
    }
};

/** Shared telemetry header used by the C++ data models. */
struct MeasurementHeader {
    /** Timestamp value in nanoseconds. */
    U64 timestamp{};
    /** Measurement sequence number. */
    U16 sequence{};
    /** Measurement flags. */
    U8 flags{};

    /** Return the whole seconds part of the timestamp. */
    constexpr U32 timestampSec() const noexcept {
        return static_cast<U32>(timestamp / NS_PER_SEC);
    }

    /** Return the nanoseconds part within the current second. */
    constexpr U32 timestampNsec() const noexcept {
        return static_cast<U32>(timestamp % NS_PER_SEC);
    }
};

/** Speed telemetry decoded from firmware function code 0x0A. */
struct SpeedReport {
    MeasurementHeader header{};
    I16 vx{};
    I16 vy{};
    I16 vz{};
};

/** IMU attitude telemetry decoded from firmware function code 0x0B. */
struct ImuAttitude {
    MeasurementHeader header{};
    I16 roll{};
    I16 pitch{};
    I16 yaw{};
};

/** Raw IMU telemetry decoded from firmware function code 0x0C. */
struct RawImuSample {
    MeasurementHeader header{};
    I16 gyro_x{};
    I16 gyro_y{};
    I16 gyro_z{};
    I16 accel_x{};
    I16 accel_y{};
    I16 accel_z{};
    I16 mag_x{};
    I16 mag_y{};
    I16 mag_z{};
};

/** Encoder telemetry decoded from firmware function code 0x0E. */
struct EncoderCounts {
    MeasurementHeader header{};
    I32 encoder_1{};
    I32 encoder_2{};
    I32 encoder_3{};
    I32 encoder_4{};
};

/** Status telemetry decoded from firmware function code 0x0F. */
struct StatusReport {
    MeasurementHeader header{};
    U8 fault_flags{};
    U16 battery_voltage{};
    U8 battery_state{};
    U8 system_enable{};
    U8 link_health{};
};

/** PID telemetry decoded from firmware function codes 0x15 and 0x16. */
struct PidReport {
    U8 index{};
    I16 kp{};
    I16 ki{};
    I16 kd{};
};

/** PID command payload encoded for transmission to the firmware. */
struct PidCommand {
    U8 index{};
    float kp{};
    float ki{};
    float kd{};
    U8 persist{};
};

/** Motion command payload encoded for transmission to the firmware. */
struct MotionCommand {
    /** Raw yaw-adjust control byte. */
    U8 yaw_adjust{};
    /** Linear velocity on X. */
    I16 vx{};
    /** Linear velocity on Y. */
    I16 vy{};
    /** Angular or rotational velocity. */
    I16 vz{};
};

} // namespace motioncore
