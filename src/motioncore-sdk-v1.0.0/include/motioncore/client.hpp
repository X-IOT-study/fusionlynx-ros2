/**
 * @file            client.hpp
 * @brief           MotionCore C++ client wrapper header
 *                  MotionCore C++ 客户端包装头文件
 * @version         1.0
 * @description     Declares the high-level C++ wrapper around the MotionCore
 *                  session API for transport binding, command sending, and
 *                  frame callbacks.
 *                  声明围绕 MotionCore 会话 API 的高级 C++ 包装层，用于绑定传输、
 *                  发送命令和接收帧回调。
 *
 * @function_list
 * 1. Client - Initialize a wrapper / 初始化包装器
 * 2. setTransport - Bind a transport backend / 绑定传输后端
 * 3. setHandlers - Register callbacks / 注册回调
 * 4. sendFrame - Send a raw frame / 发送原始帧
 * 5. poll - Poll the bound transport / 轮询绑定的传输层
 * 6. feedBytes - Feed bytes into the decoder / 向解码器喂入字节
 * 7. sendTimeSync - Send a time sync command / 发送时间同步命令
 * 8. sendHeartbeat - Send a heartbeat command / 发送心跳命令
 * 9. requestData - Request a report payload / 请求遥测数据
 * 10. setMotorPid - Send a motor PID command / 发送电机 PID 命令
 * 11. setYawPid - Send a yaw PID command / 发送航向 PID 命令
 * 12. setMotion - Send a motion command / 发送运动控制命令
 * 13. resetState - Reset runtime state / 重置运行状态
 * 14. resetFlash - Reset persisted settings / 重置持久化参数
 * 15. setAutoReport - Configure auto report / 配置自动上报
 * 16. setBeep - Trigger the buzzer / 触发蜂鸣器
 * 17. testMode - Toggle test mode / 切换测试模式
 * 18. clearYaw - Clear yaw state / 清零航向
 * 19. native - Access the native handle / 访问原生句柄
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#pragma once

#include <cstddef>
#include <functional>
#include <utility>

extern "C" {
#include <motioncore/mc_client.h>
#include <motioncore/mc_protocol.h>
}
#include <motioncore/types.hpp>

namespace motioncore {

/** High-level C++ wrapper around the MotionCore session API. */
class Client {
public:
    using FrameHandler = std::function<void(const mc_frame_t &)>;
    using ErrorHandler = std::function<void(int)>;
    using SpeedHandler = std::function<void(const SpeedReport &)>;
    using ImuHandler = std::function<void(const ImuAttitude &)>;
    using RawImuHandler = std::function<void(const RawImuSample &)>;
    using EncoderHandler = std::function<void(const EncoderCounts &)>;
    using StatusHandler = std::function<void(const StatusReport &)>;

    /** Default-construct the wrapper and initialize the native client state. */
    Client();

    /**
     * Bind a transport backend to the wrapper.
     *
     * @param transport Transport handle to bind.
     * @return 0 on success, or a negative error code.
     */
    int setTransport(mc_transport_t *transport);

    /**
     * Register frame and error handlers.
     *
     * @param onFrame Callback invoked for decoded frames.
     * @param onError Callback invoked for parse or transport errors.
     * @return 0 on success, or a negative error code.
     */
    int setHandlers(FrameHandler onFrame, ErrorHandler onError);

    /**
     * Send one encoded frame through the bound transport.
     *
     * @param frame Encoded frame bytes.
     * @param frame_len Frame length in bytes.
     * @return 0 on success, or a negative error code.
     */
    int sendFrame(const mc_u8 *frame, std::size_t frame_len);

    /**
     * Poll the transport once and dispatch any complete frame.
     *
     * @return 1 when bytes were read and processed, 0 when no data was
     *         available, or a negative error code on failure.
     */
    int poll();

    /**
     * Feed raw bytes into the decoder.
     *
     * @param bytes Input byte sequence.
     * @param length Number of bytes to append.
     * @return 0 when bytes were buffered successfully, or a negative error
     *         code when the input is invalid, the buffer overflows, or frame
     *         dispatch fails.
     */
    int feedBytes(const mc_u8 *bytes, std::size_t length);

    /**
     * Send a firmware time synchronization command.
     *
     * @param timestamp Host timestamp in nanoseconds.
     * @return 0 on success, or a negative error code.
     */
    int sendTimeSync(mc_u64 timestamp);

    /** Send a firmware heartbeat command. */
    int sendHeartbeat();

    /**
     * Request a telemetry payload by function code and index.
     *
     * @param request_function_code Target function code.
     * @param request_index Optional report index.
     * @return 0 on success, or a negative error code.
     */
    int requestData(mc_u8 request_function_code, mc_u8 request_index);

    /**
     * Send a motor PID command.
     *
     * @param command PID command values in host units.
     * @return 0 on success, or a negative error code.
     */
    int setMotorPid(const PidCommand &command);

    /**
     * Send a yaw PID command.
     *
     * @param command PID command values in host units.
     * @return 0 on success, or a negative error code.
     */
    int setYawPid(const PidCommand &command);

    /**
     * Send a motion command for the drive base.
     *
     * @param command Motion command in host units.
     * @return 0 on success, or a negative error code.
     */
    int setMotion(const MotionCommand &command);

    /** Reset the firmware runtime state. */
    int resetState();

    /** Reset persisted flash parameters. */
    int resetFlash();

    /**
     * Configure the auto-report state.
     *
     * @param state Auto-report state byte.
     * @return 0 on success, or a negative error code.
     */
    int setAutoReport(mc_u8 state);

    /**
     * Trigger the buzzer for a given duration.
     *
     * @param duration_ms Duration in milliseconds.
     * @return 0 on success, or a negative error code.
     */
    int setBeep(mc_u16 duration_ms);

    /**
     * Toggle firmware test mode.
     *
     * @param enabled Non-zero to enable test mode.
     * @return 0 on success, or a negative error code.
     */
    int testMode(mc_u8 enabled);

    /** Clear the firmware yaw state. */
    int clearYaw();

    /** Access the underlying native client handle. */
    mc_client_t *native() { return &client_; }
    /** Access the underlying native client handle. */
    const mc_client_t *native() const { return &client_; }

private:
    static void dispatchFrame(void *user, const mc_frame_t *frame);
    static void dispatchError(void *user, int error_code);

    int sendCommand(mc_u8 function_code, const mc_u8 *payload, std::size_t payload_len);

    mc_client_t client_{};
    FrameHandler onFrame_;
    ErrorHandler onError_;
};

} // namespace motioncore
