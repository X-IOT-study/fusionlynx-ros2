from motioncore import Client, EncoderCounts, ImuAttitude, MeasurementHeader, MotionCommand, PidCommand, PidReport, RawImuSample, SpeedReport, StatusReport, Timestamp, TransportSerial

header = MeasurementHeader(timestamp=1, sequence=2, flags=3)
speed = SpeedReport(header=header, vx=4, vy=5, vz=6)
imu = ImuAttitude(header=header, roll=7, pitch=8, yaw=9)
raw = RawImuSample(
    header=header,
    gyro_x=10,
    gyro_y=11,
    gyro_z=12,
    accel_x=13,
    accel_y=14,
    accel_z=15,
    mag_x=16,
    mag_y=17,
    mag_z=18,
)
enc = EncoderCounts(header=header, encoder_1=19, encoder_2=20, encoder_3=21, encoder_4=22)
status = StatusReport(header=header, fault_flags=23, battery_voltage=24, battery_state=25, system_enable=26, link_health=27)
pid_report = PidReport(index=5, kp=1, ki=2, kd=3)
pid = PidCommand(index=5, kp=1.0, ki=2.0, kd=3.0, persist=1)
stamp = Timestamp(value=7)
client = Client()
transport = TransportSerial()
motion = MotionCommand(vx=1, vy=2, vz=3, yaw_adjust=4)
print(speed)
print(imu)
print(raw)
print(enc)
print(status)
print(pid_report)
print(pid)
print(stamp)
print(client.request_motor_pid(2))
print(client.request_yaw_pid())
print(client.send_time_sync(stamp))
print(client.set_motion(motion))
print(transport)
