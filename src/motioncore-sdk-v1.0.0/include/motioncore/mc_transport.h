/**
 * @file            mc_transport.h
 * @brief           MotionCore transport abstraction header
 *                  MotionCore 传输抽象头文件
 * @version         1.0
 * @description     Declares the callback table used to adapt serial, socket,
 *                  or other byte-stream transports into the MotionCore client.
 *                  声明用于把串口、套接字或其他字节流传输适配到 MotionCore 客户端的回调表。
 *
 * @function_list
 * 1. mc_transport_init - Initialize a transport handle / 初始化传输句柄
 * 2. mc_transport_read - Read bytes through the transport / 通过传输层读取字节
 * 3. mc_transport_write - Write bytes through the transport / 通过传输层写入字节
 * 4. mc_transport_close - Close the transport backend / 关闭传输后端
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#ifndef MOTIONCORE_MC_TRANSPORT_H
#define MOTIONCORE_MC_TRANSPORT_H

#include <stddef.h>

#include <motioncore/mc_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Transport callback table used to adapt a byte-stream backend. */
typedef struct {
    /** Read up to buffer_len bytes and return the byte count actually read. */
    size_t (*read)(void *user, mc_u8 *buffer, size_t buffer_len);
    /** Write up to buffer_len bytes and return the byte count actually written. */
    size_t (*write)(void *user, const mc_u8 *buffer, size_t buffer_len);
    /** Close the backend when the transport is torn down. */
    void (*close)(void *user);
    /** Opaque pointer passed back to every callback. */
    void *user;
} mc_transport_ops_t;

/**
 * Transport handle used by the session layer.
 */
typedef struct mc_transport {
    /** Registered transport callbacks. */
    mc_transport_ops_t ops;
} mc_transport_t;

/**
 * Initialize a transport handle.
 *
 * @param transport Transport handle to initialize.
 * @param ops Callback table to copy into the transport handle.
 * @return 0 on success, or -1 when transport or ops is NULL.
 */
int mc_transport_init(mc_transport_t *transport, const mc_transport_ops_t *ops);

/**
 * Read bytes through the configured transport callback.
 *
 * @param transport Transport handle.
 * @param buffer Destination buffer.
 * @param buffer_len Destination buffer size in bytes.
 * @return Number of bytes read, or 0 when no data is available or the
 *         transport is not ready.
 */
size_t mc_transport_read(mc_transport_t *transport, mc_u8 *buffer, size_t buffer_len);

/**
 * Write bytes through the configured transport callback.
 *
 * @param transport Transport handle.
 * @param buffer Source buffer.
 * @param buffer_len Source buffer size in bytes.
 * @return Number of bytes written, or 0 when the transport is not ready.
 */
size_t mc_transport_write(mc_transport_t *transport, const mc_u8 *buffer, size_t buffer_len);

/**
 * Close the configured transport backend if a close callback is present.
 *
 * @param transport Transport handle.
 */
void mc_transport_close(mc_transport_t *transport);

#ifdef __cplusplus
}
#endif

#endif
