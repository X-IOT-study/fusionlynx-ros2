#!/usr/bin/env python3

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

import numpy as np
import matplotlib.pyplot as plt


# ==========================
# 配置
# ==========================

bag_path = "imu_compare_static"


# ROS2 Humble 类型系统
typestore = get_typestore(Stores.ROS2_HUMBLE)


# 保存数据
unilidar = []
motioncore = []


# ==========================
# 读取bag
# ==========================

print("Reading bag...")


with Reader(bag_path) as reader:

    for connection, timestamp, rawdata in reader.messages():

        if connection.topic == "/unilidar/imu":

            msg = typestore.deserialize_cdr(
                rawdata,
                connection.msgtype
            )

            unilidar.append(
                [
                    msg.header.stamp.sec +
                    msg.header.stamp.nanosec * 1e-9,

                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,

                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]
            )


        elif connection.topic == "/motioncore/imu/data":

            msg = typestore.deserialize_cdr(
                rawdata,
                connection.msgtype
            )

            motioncore.append(
                [
                    msg.header.stamp.sec +
                    msg.header.stamp.nanosec * 1e-9,

                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,

                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]
            )



unilidar = np.array(unilidar)
motioncore = np.array(motioncore)



# ==========================
# 基本信息
# ==========================


print("\n========== DATA SIZE ==========")

print(
    "UniLidar IMU:",
    unilidar.shape
)

print(
    "MotionCore IMU:",
    motioncore.shape
)



# ==========================
# 频率计算
# ==========================

def calc_rate(data):

    if len(data)<2:
        return 0

    dt=np.diff(data[:,0])

    return 1.0/np.mean(dt)



print("\n========== FREQUENCY ==========")

print(
    "UniLidar rate:",
    calc_rate(unilidar),
    "Hz"
)


print(
    "MotionCore rate:",
    calc_rate(motioncore),
    "Hz"
)



# ==========================
# bias分析
# ==========================

def imu_report(name,data):

    gyro=data[:,1:4]
    acc=data[:,4:7]


    print("\n======================")
    print(name)
    print("======================")


    print("gyro bias(rad/s):")
    print(
        np.mean(gyro,axis=0)
    )


    print("\nacc mean(m/s^2):")
    print(
        np.mean(acc,axis=0)
    )


    norm=np.linalg.norm(
        acc,
        axis=1
    )


    print("\nacc norm:")
    print(
        "mean:",
        np.mean(norm)
    )

    print(
        "std:",
        np.std(norm)
    )



if len(unilidar)>0:
    imu_report(
        "UniLidar",
        unilidar
    )


if len(motioncore)>0:
    imu_report(
        "MotionCore",
        motioncore
    )



# ==========================
# 绘图
# ==========================


def plot_gyro(data,name):

    t=data[:,0]-data[0,0]


    plt.figure(figsize=(10,5))

    plt.plot(
        t,
        data[:,1],
        label="gyro x"
    )

    plt.plot(
        t,
        data[:,2],
        label="gyro y"
    )

    plt.plot(
        t,
        data[:,3],
        label="gyro z"
    )


    plt.title(
        name+" gyro"
    )

    plt.xlabel(
        "time(s)"
    )

    plt.ylabel(
        "rad/s"
    )

    plt.grid()

    plt.legend()

    plt.show()



def plot_acc(data,name):

    t=data[:,0]-data[0,0]


    plt.figure(figsize=(10,5))


    plt.plot(
        t,
        data[:,4],
        label="acc x"
    )


    plt.plot(
        t,
        data[:,5],
        label="acc y"
    )


    plt.plot(
        t,
        data[:,6],
        label="acc z"
    )


    plt.title(
        name+" acceleration"
    )

    plt.xlabel(
        "time(s)"
    )

    plt.ylabel(
        "m/s^2"
    )

    plt.grid()

    plt.legend()

    plt.show()



if len(unilidar)>0:

    plot_gyro(
        unilidar,
        "UniLidar"
    )

    plot_acc(
        unilidar,
        "UniLidar"
    )


if len(motioncore)>0:

    plot_gyro(
        motioncore,
        "MotionCore"
    )

    plot_acc(
        motioncore,
        "MotionCore"
    )
