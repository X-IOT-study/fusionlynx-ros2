/**
 * @file            transport.hpp
 * @brief           MotionCore C++ transport wrapper header
 *                  MotionCore C++ 传输包装头文件
 * @version         1.0
 * @description     Declares a thin C++ wrapper around the MotionCore
 *                  transport abstraction and its callback hooks.
 *                  声明围绕 MotionCore 传输抽象及其回调钩子的轻量 C++ 包装层。
 *
 * @function_list
 * 1. Transport - Initialize a wrapper / 初始化包装器
 * 2. bind - Bind callback handlers / 绑定回调处理器
 * 3. native - Access the native transport / 访问原生传输句柄
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

#include <motioncore/mc_transport.h>

namespace motioncore {

/** Thin C++ wrapper around mc_transport_t for callback-based adapters. */
class Transport {
public:
    using ReadHandler = std::function<std::size_t(mc_u8 *, std::size_t)>;
    using WriteHandler = std::function<std::size_t(const mc_u8 *, std::size_t)>;
    using CloseHandler = std::function<void()>;

    /** Default-construct an empty transport wrapper. */
    Transport() = default;

    /**
     * Bind callback handlers to the transport wrapper.
     *
     * @param read Read callback used by the transport.
     * @param write Write callback used by the transport.
     * @param close Optional close callback.
     * @return 0 on success, or a negative error code.
     */
    int bind(ReadHandler read, WriteHandler write, CloseHandler close = {});

    /** Access the native transport handle. */
    mc_transport_t *native() { return &transport_; }
    /** Access the native transport handle. */
    const mc_transport_t *native() const { return &transport_; }

private:
    static std::size_t dispatchRead(void *user, mc_u8 *buffer, std::size_t buffer_len);
    static std::size_t dispatchWrite(void *user, const mc_u8 *buffer, std::size_t buffer_len);
    static void dispatchClose(void *user);

    struct Callbacks {
        ReadHandler read;
        WriteHandler write;
        CloseHandler close;
    } callbacks_;

    mc_transport_t transport_{};
};

} // namespace motioncore
