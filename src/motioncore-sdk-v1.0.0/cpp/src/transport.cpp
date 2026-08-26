#include <motioncore/transport.hpp>

namespace motioncore {

int Transport::bind(ReadHandler read, WriteHandler write, CloseHandler close) {
    callbacks_.read = std::move(read);
    callbacks_.write = std::move(write);
    callbacks_.close = std::move(close);
    mc_transport_ops_t ops{};
    ops.read = &Transport::dispatchRead;
    ops.write = &Transport::dispatchWrite;
    ops.close = &Transport::dispatchClose;
    ops.user = &callbacks_;
    return mc_transport_init(&transport_, &ops);
}

std::size_t Transport::dispatchRead(void *user, mc_u8 *buffer, std::size_t buffer_len) {
    auto *callbacks = static_cast<Callbacks *>(user);
    return callbacks != nullptr && callbacks->read ? callbacks->read(buffer, buffer_len) : 0u;
}

std::size_t Transport::dispatchWrite(void *user, const mc_u8 *buffer, std::size_t buffer_len) {
    auto *callbacks = static_cast<Callbacks *>(user);
    return callbacks != nullptr && callbacks->write ? callbacks->write(buffer, buffer_len) : 0u;
}

void Transport::dispatchClose(void *user) {
    auto *callbacks = static_cast<Callbacks *>(user);
    if (callbacks != nullptr && callbacks->close) {
        callbacks->close();
    }
}

} // namespace motioncore
