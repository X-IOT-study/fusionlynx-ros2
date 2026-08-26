#include <stdio.h>

#include <motioncore/protocol.h>

static void dump_bytes(const mc_u8 *bytes, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        printf("%02X%s", bytes[i], (i + 1u == length) ? "" : " ");
    }
    printf("\n");
}

int main(void) {
    const mc_u64 timestamp = 0x0102030405060708ull;
    const mc_u8 pid_request[] = {MC_FUNC_REPORT_MOTOR_PID, 0x02u};
    const mc_u8 yaw_request[] = {MC_FUNC_REPORT_YAW_PID, 0x05u};
    mc_u8 frame[32] = {0};
    mc_u8 time_payload[8] = {0};
    mc_u8 pid_payload[8] = {0};
    size_t frame_len;
    size_t payload_len;

    payload_len = mc_encode_time_sync_payload(time_payload, sizeof(time_payload), timestamp);
    if (payload_len != sizeof(time_payload)) {
        return 1;
    }
    frame_len = mc_frame_encode(MC_DEVICE_ID_TX, MC_FUNC_TIME_SYNC, time_payload, payload_len, frame, sizeof(frame));
    if (frame_len == 0u) {
        return 2;
    }
    printf("time_sync: ");
    dump_bytes(frame, frame_len);

    payload_len = mc_encode_request_pid_payload(pid_payload, sizeof(pid_payload), MC_FUNC_REPORT_MOTOR_PID, 2u);
    if (payload_len != sizeof(pid_request)) {
        return 3;
    }
    frame_len = mc_frame_encode(MC_DEVICE_ID_TX, MC_FUNC_REQUEST_DATA, pid_payload, payload_len, frame, sizeof(frame));
    if (frame_len == 0u) {
        return 4;
    }
    printf("motor_pid_request: ");
    dump_bytes(frame, frame_len);

    payload_len = mc_encode_request_pid_payload(pid_payload, sizeof(pid_payload), MC_FUNC_REPORT_YAW_PID, 5u);
    if (payload_len != sizeof(yaw_request)) {
        return 5;
    }
    frame_len = mc_frame_encode(MC_DEVICE_ID_TX, MC_FUNC_REQUEST_DATA, pid_payload, payload_len, frame, sizeof(frame));
    if (frame_len == 0u) {
        return 6;
    }
    printf("yaw_pid_request: ");
    dump_bytes(frame, frame_len);

    return 0;
}
