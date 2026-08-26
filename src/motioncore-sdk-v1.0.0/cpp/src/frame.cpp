#include <motioncore/frame.hpp>

namespace motioncore {

Frame Frame::encode(mc_u8 device_id, mc_u8 function_code, const mc_u8 *payload, std::size_t payload_len) {
    Frame frame;
    mc_frame_encode(device_id, function_code, payload, payload_len, reinterpret_cast<mc_u8 *>(&frame.frame_), sizeof(frame.frame_));
    return frame;
}

std::size_t Frame::decode(const mc_u8 *bytes, std::size_t length) {
    return mc_frame_decode(bytes, length, &frame_) == 0 ? length : 0u;
}

} // namespace motioncore
