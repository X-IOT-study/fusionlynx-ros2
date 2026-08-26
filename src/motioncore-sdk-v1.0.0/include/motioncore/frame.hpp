/**
 * @file            frame.hpp
 * @brief           MotionCore C++ frame wrapper header
 *                  MotionCore C++ 帧包装头文件
 * @version         1.0
 * @description     Declares a thin C++ wrapper around the raw MotionCore
 *                  frame container and codec helpers.
 *                  声明围绕原始 MotionCore 帧容器和编解码辅助函数的轻量 C++ 包装层。
 *
 * @function_list
 * 1. Frame - Initialize a wrapper / 初始化包装器
 * 2. encode - Encode a frame / 编码帧
 * 3. decode - Decode a frame / 解码帧
 * 4. native - Access the native frame / 访问原生帧
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
#include <vector>

extern "C" {
#include <motioncore/mc_frame.h>
#include <motioncore/mc_protocol.h>
}

namespace motioncore {

/** Thin C++ wrapper around mc_frame_t for encode/decode convenience. */
class Frame {
public:
    /** Default-construct an empty frame wrapper. */
    Frame() = default;

    /**
     * Encode a frame from raw bytes.
     *
     * @param device_id Protocol device identifier.
     * @param function_code Protocol function code.
     * @param payload Payload bytes to encode.
     * @param payload_len Number of payload bytes.
     * @return Encoded frame wrapper. If encoding fails, the returned wrapper
     *         remains default-initialized.
     */
    static Frame encode(mc_u8 device_id, mc_u8 function_code, const mc_u8 *payload, std::size_t payload_len);

    /**
     * Decode a raw frame into the wrapper.
     *
     * @param bytes Raw frame bytes.
     * @param length Frame length in bytes.
     * @return Input length on success, or 0 on failure.
     */
    std::size_t decode(const mc_u8 *bytes, std::size_t length);

    /** Access the native frame representation. */
    const mc_frame_t &native() const { return frame_; }

private:
    mc_frame_t frame_{};
};

} // namespace motioncore
