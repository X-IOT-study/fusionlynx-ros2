#include <iostream>

#include <motioncore/client.hpp>
#include <motioncore/frame.hpp>

int main() {
    motioncore::Client client;
    motioncore::PidCommand motor_pid{2u, 1.0f, 2.0f, 3.0f, 1u};
    motioncore::PidCommand yaw_pid{5u, 4.0f, 5.0f, 6.0f, 0u};
    motioncore::MotionCommand motion{4u, 10, 20, 30};
    const mc_u64 timestamp = 0x0102030405060708ull;

    std::cout << "request motor pid: " << client.requestData(MC_FUNC_REPORT_MOTOR_PID, 2u) << std::endl;
    std::cout << "request yaw pid: " << client.requestData(MC_FUNC_REPORT_YAW_PID, 5u) << std::endl;
    std::cout << "set motor pid: " << client.setMotorPid(motor_pid) << std::endl;
    std::cout << "set yaw pid: " << client.setYawPid(yaw_pid) << std::endl;
    std::cout << "set motion: " << client.setMotion(motion) << std::endl;
    std::cout << "time sync: " << client.sendTimeSync(timestamp) << std::endl;
    std::cout << "heartbeat: " << client.sendHeartbeat() << std::endl;
    std::cout << "clear yaw: " << client.clearYaw() << std::endl;
    return 0;
}
