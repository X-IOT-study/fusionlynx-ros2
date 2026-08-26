/**
 * @file            mc_frame.h
 * @brief           MotionCore frame codec header
 *                  MotionCore 帧编解码头文件
 * @version         1.0
 * @description     Declares the raw frame container and the helpers used to
 *                  validate, encode, and decode MotionCore wire frames.
 *                  声明用于校验、编码和解码 MotionCore 线协议帧的原始帧容器和辅助函数。
 *
 * @function_list
 * 1. mc_checksum_compute - Compute the frame checksum / 计算帧校验和
 * 2. mc_frame_encode - Encode a wire frame / 编码线协议帧
 * 3. mc_frame_validate - Validate a wire frame / 校验线协议帧
 * 4. mc_frame_decode - Decode a wire frame / 解码线协议帧
 *
 * @history
 * 1. Date: 2026/7/21
 *    Author: XvWenSheng
 *    Modification: Standardized public API documentation.
 *
 * @CopyRight (c) 2025-2027 2453395023[at]qq.com All Right Reserved
 */

#ifndef MOTIONCORE_MC_FRAME_H
#define MOTIONCORE_MC_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include <motioncore/mc_types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MC_FRAME_HEADER 0xFF
#define MC_DEVICE_ID_TX 0xFB
#define MC_DEVICE_ID_RX 0xFC
#define MC_FRAME_MAX_PAYLOAD 255u

/**
 * Raw MotionCore frame container used by the codec helpers.
 */
typedef struct {
    /** Device identifier byte. */
    mc_u8 device_id;
    /** Protocol function code. */
    mc_u8 function_code;
    /** Payload bytes extracted from or written into the frame. */
    mc_u8 payload[MC_FRAME_MAX_PAYLOAD];
    /** Number of valid payload bytes. */
    size_t payload_len;
} mc_frame_t;

/**
 * Compute the MotionCore checksum over a frame segment.
 *
 * @param bytes Input byte sequence.
 * @param length Number of bytes to include.
 * @return Low 8 bits of the additive checksum.
 */
mc_u8 mc_checksum_compute(const mc_u8 *bytes, size_t length);

/**
 * Encode a MotionCore frame into a caller-provided buffer.
 *
 * @param device_id Protocol device identifier.
 * @param function_code Protocol function code.
 * @param payload Payload bytes to encode.
 * @param payload_len Number of payload bytes.
 * @param out Output buffer.
 * @param out_len Output buffer size in bytes.
 * @return Encoded frame length on success, or 0 on failure.
 */
size_t mc_frame_encode(mc_u8 device_id, mc_u8 function_code, const mc_u8 *payload, size_t payload_len, mc_u8 *out, size_t out_len);

/**
 * Validate a raw MotionCore frame without decoding it into a structure.
 *
 * @param frame Raw frame bytes.
 * @param frame_len Frame length in bytes.
 * @return 0 on success, negative error code on failure.
 */
int mc_frame_validate(const mc_u8 *frame, size_t frame_len);

/**
 * Decode a MotionCore frame from raw bytes.
 *
 * @param frame Raw frame bytes.
 * @param frame_len Frame length in bytes.
 * @param out_frame Destination frame structure.
 * @return 0 on success, negative error code on failure.
 */
int mc_frame_decode(const mc_u8 *frame, size_t frame_len, mc_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif
