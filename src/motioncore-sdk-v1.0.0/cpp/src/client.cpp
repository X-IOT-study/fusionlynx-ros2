#include <motioncore/client.hpp>

namespace motioncore {

Client::Client() {
    mc_client_init(&client_);
}

int Client::setTransport(mc_transport_t *transport) {
    return mc_client_set_transport(&client_, transport);
}

int Client::setHandlers(FrameHandler onFrame, ErrorHandler onError) {
    onFrame_ = std::move(onFrame);
    onError_ = std::move(onError);
    mc_client_handlers_t handlers{};
    handlers.user = this;
    handlers.on_frame = &Client::dispatchFrame;
    handlers.on_error = &Client::dispatchError;
    return mc_client_set_handlers(&client_, &handlers);
}

int Client::sendFrame(const mc_u8 *frame, std::size_t frame_len) {
    return mc_client_send_frame(&client_, frame, frame_len);
}

int Client::poll() {
    return mc_client_poll(&client_);
}

int Client::feedBytes(const mc_u8 *bytes, std::size_t length) {
    return mc_client_feed_bytes(&client_, bytes, length);
}

int Client::sendCommand(mc_u8 function_code, const mc_u8 *payload, std::size_t payload_len) {
    mc_u8 frame[64u] = {0};
    const std::size_t frame_len = mc_frame_encode(
      MC_DEVICE_ID_RX, function_code, payload, payload_len, frame, sizeof(frame));
    return frame_len == 0u ? -1 : mc_client_send_frame(&client_, frame, frame_len);
}

int Client::sendTimeSync(mc_u64 timestamp) {
    mc_u8 payload[8u] = {0};
    const std::size_t payload_len = mc_encode_time_sync_payload(payload, sizeof(payload), timestamp);
    return payload_len == 0u ? -1 : sendCommand(MC_FUNC_TIME_SYNC, payload, payload_len);
}

int Client::sendHeartbeat() {
    mc_u8 payload[1u] = {0};
    const std::size_t payload_len = mc_encode_heartbeat_payload(payload, sizeof(payload));
    return payload_len == 0u ? -1 : sendCommand(MC_FUNC_HEARTBEAT_REQUEST, payload, payload_len);
}

int Client::requestData(mc_u8 request_function_code, mc_u8 request_index) {
    mc_u8 payload[2u] = {0};
    const std::size_t payload_len = mc_encode_request_pid_payload(
      payload, sizeof(payload), request_function_code, request_index);
    return payload_len == 0u ? -1 : sendCommand(MC_FUNC_REQUEST_DATA, payload, payload_len);
}

int Client::setMotorPid(const PidCommand &command) {
    mc_u8 payload[8u] = {0};
    const mc_i16 kp = (mc_i16)(command.kp * 1000.0f);
    const mc_i16 ki = (mc_i16)(command.ki * 1000.0f);
    const mc_i16 kd = (mc_i16)(command.kd * 1000.0f);
    payload[0] = command.index;
    payload[1] = (mc_u8)(kp & 0xFFu);
    payload[2] = (mc_u8)((kp >> 8) & 0xFFu);
    payload[3] = (mc_u8)(ki & 0xFFu);
    payload[4] = (mc_u8)((ki >> 8) & 0xFFu);
    payload[5] = (mc_u8)(kd & 0xFFu);
    payload[6] = (mc_u8)((kd >> 8) & 0xFFu);
    payload[7] = command.persist;
    return sendCommand(0x13u, payload, sizeof(payload));
}

int Client::setYawPid(const PidCommand &command) {
    mc_u8 payload[8u] = {0};
    const mc_i16 kp = (mc_i16)(command.kp * 1000.0f);
    const mc_i16 ki = (mc_i16)(command.ki * 1000.0f);
    const mc_i16 kd = (mc_i16)(command.kd * 1000.0f);
    payload[0] = command.index;
    payload[1] = (mc_u8)(kp & 0xFFu);
    payload[2] = (mc_u8)((kp >> 8) & 0xFFu);
    payload[3] = (mc_u8)(ki & 0xFFu);
    payload[4] = (mc_u8)((ki >> 8) & 0xFFu);
    payload[5] = (mc_u8)(kd & 0xFFu);
    payload[6] = (mc_u8)((kd >> 8) & 0xFFu);
    payload[7] = command.persist;
    return sendCommand(0x14u, payload, sizeof(payload));
}

int Client::setMotion(const MotionCommand &command) {
    mc_u8 payload[7u] = {0};
    const std::size_t payload_len = mc_encode_motion_command(
      payload, sizeof(payload), command.vx, command.vy, command.vz, command.yaw_adjust);
    return payload_len == 0u ? -1 : sendCommand(0x12u, payload, payload_len);
}

int Client::resetState() {
    mc_u8 payload[1u] = {0xA5u};
    return sendCommand(0x0Du, payload, sizeof(payload));
}

int Client::resetFlash() {
    mc_u8 payload[1u] = {0xA5u};
    return sendCommand(0xA0u, payload, sizeof(payload));
}

int Client::setAutoReport(mc_u8 state) {
    mc_u8 payload[2u] = {state, 0xA5u};
    return sendCommand(0x01u, payload, sizeof(payload));
}

int Client::setBeep(mc_u16 duration_ms) {
    mc_u8 payload[2u] = {(mc_u8)(duration_ms & 0xFFu), (mc_u8)((duration_ms >> 8) & 0xFFu)};
    return sendCommand(0x02u, payload, sizeof(payload));
}

int Client::testMode(mc_u8 enabled) {
    mc_u8 payload[1u] = {enabled};
    return sendCommand(0xA1u, payload, sizeof(payload));
}

int Client::clearYaw() {
    mc_u8 payload[1u] = {0xA5u};
    return sendCommand(0xA2u, payload, sizeof(payload));
}

void Client::dispatchFrame(void *user, const mc_frame_t *frame) {
    auto *self = static_cast<Client *>(user);
    if (self != nullptr && self->onFrame_) {
        self->onFrame_(*frame);
    }
}

void Client::dispatchError(void *user, int error_code) {
    auto *self = static_cast<Client *>(user);
    if (self != nullptr && self->onError_) {
        self->onError_(error_code);
    }
}

} // namespace motioncore
