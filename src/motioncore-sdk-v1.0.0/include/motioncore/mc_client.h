/**
 * @file            mc_client.h
 * @brief           MotionCore session/client API header
 *                  MotionCore 会话/客户端 API 头文件
 * @version         1.0
 * @description     Declares the session handle and callbacks used to bind a
 *                  transport, send frames, and consume parsed protocol frames.
 *                  声明用于绑定传输层、发送帧以及消费解析后协议帧的会话句柄和回调。
 *
 * @function_list
 * 1. mc_client_init - Initialize a client handle / 初始化客户端句柄
 * 2. mc_client_set_transport - Bind a transport backend / 绑定传输后端
 * 3. mc_client_set_handlers - Register frame/error callbacks / 注册帧和错误回调
 * 4. mc_client_send_frame - Send a raw frame through the transport / 通过传输层发送原始帧
 * 5. mc_client_poll - Read and dispatch buffered transport data / 读取并派发传输数据
 * 6. mc_client_feed_bytes - Feed raw bytes into the decoder / 向解码器喂入原始字节
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#ifndef MOTIONCORE_MC_CLIENT_H
#define MOTIONCORE_MC_CLIENT_H

#include <stddef.h>

#include <motioncore/mc_frame.h>
#include <motioncore/mc_protocol.h>
#include <motioncore/mc_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Frame and error callbacks used by a MotionCore client session. */
typedef struct {
    /** Invoked when a complete frame has been decoded. */
    void (*on_frame)(void *user, const mc_frame_t *frame);
    /** Invoked when parsing or transport operations fail. */
    void (*on_error)(void *user, int error_code);
    /** Opaque pointer passed back to the callbacks. */
    void *user;
} mc_client_handlers_t;

/**
 * Session/client handle used to bind a transport and decode incoming bytes.
 */
typedef struct mc_client {
    /** Bound transport backend used for reads and writes. */
    mc_transport_t *transport;
    /** Registered application callbacks. */
    mc_client_handlers_t handlers;
    /** Internal byte buffer used while assembling frames. */
    mc_u8 buffer[512u];
    /** Number of valid bytes currently stored in buffer. */
    size_t buffered_len;
} mc_client_t;

/**
 * Initialize a client handle.
 *
 * @param client Client handle to initialize.
 * @return 0 on success, or -1 when client is NULL.
 */
int mc_client_init(mc_client_t *client);

/**
 * Bind a transport backend to the client.
 *
 * @param client Client handle.
 * @param transport Transport backend to bind.
 * @return 0 on success, or -1 when client is NULL.
 */
int mc_client_set_transport(mc_client_t *client, mc_transport_t *transport);

/**
 * Register frame and error callbacks for the client.
 *
 * @param client Client handle.
 * @param handlers Callback table to copy into the client.
 * @return 0 on success, or -1 when client or handlers is NULL.
 */
int mc_client_set_handlers(mc_client_t *client, const mc_client_handlers_t *handlers);

/**
 * Send one encoded frame through the bound transport.
 *
 * @param client Client handle.
 * @param frame Encoded frame bytes.
 * @param frame_len Frame length in bytes.
 * @return 0 on success, -1 when client or transport is NULL, -2 when the
 *         transport reports a short write.
 */
int mc_client_send_frame(mc_client_t *client, const mc_u8 *frame, size_t frame_len);

/**
 * Read bytes from the transport and dispatch any complete frame.
 *
 * @param client Client handle.
 * @return 1 when bytes were read and processed, 0 when no data was available,
 *         or a negative error code on failure.
 */
int mc_client_poll(mc_client_t *client);

/**
 * Feed raw bytes into the frame buffer and dispatch complete frames.
 *
 * @param client Client handle.
 * @param bytes Input byte sequence.
 * @param length Number of bytes to append.
 * @return 0 when bytes were buffered successfully, or a negative error code
 *         when the input is invalid, the buffer overflows, or frame dispatch
 *         fails.
 */
int mc_client_feed_bytes(mc_client_t *client, const mc_u8 *bytes, size_t length);

#ifdef __cplusplus
}
#endif

#endif
