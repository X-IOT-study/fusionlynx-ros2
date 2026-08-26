#!/usr/bin/env python3

import os
import subprocess

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_xyz(value):
    return " ".join(str(item) for item in value)


def _load_robot_config(params_file):
    with open(params_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("fusionlynx_description")
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true", "yes")
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in ("1", "true", "yes")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui").perform(context).lower() in ("1", "true", "yes")

    cfg = _load_robot_config(params_file)
    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    rviz_config = cfg.get("rviz_config", os.path.join(pkg_share, "config", "display.rviz"))

    robot_name = cfg.get("robot_name", "fusionlynx")
    xacro_cmd = [
        "xacro",
        xacro_file,
        f"robot_name:={robot_name}",
        f"base_link:={cfg.get('base_link', 'base_link')}",
        f"imu_link:={cfg.get('imu_link', 'imu_link')}",
        f"mag_link:={cfg.get('mag_link', 'mag_link')}",
        f"body_link:={cfg.get('body_link', 'body')}",
        f"lidar_link:={cfg.get('lidar_link', 'lidar_link')}",
        f"camera_link:={cfg.get('camera_link', 'camera_link')}",
        f"base_length:={cfg.get('base_length', 0.52)}",
        f"base_width:={cfg.get('base_width', 0.42)}",
        f"base_height:={cfg.get('base_height', 0.18)}",
        f"wheel_radius:={cfg.get('wheel_radius', 0.055)}",
        f"wheel_width:={cfg.get('wheel_width', 0.03)}",
        f"wheel_base:={cfg.get('wheel_base', 0.2141)}",
        f"track_width:={cfg.get('track_width', 0.2141)}",
        f"wheel_fl_xyz:={_as_xyz(cfg.get('wheel_fl_xyz', [0.11, 0.1035, -0.043073]))}",
        f"wheel_fr_xyz:={_as_xyz(cfg.get('wheel_fr_xyz', [0.11, -0.1035, -0.043073]))}",
        f"wheel_rl_xyz:={_as_xyz(cfg.get('wheel_rl_xyz', [-0.11, 0.1035, -0.039499]))}",
        f"wheel_rr_xyz:={_as_xyz(cfg.get('wheel_rr_xyz', [-0.11, -0.1035, -0.039499]))}",
        f"imu_xyz:={_as_xyz(cfg.get('imu_xyz', [0.006176, 0.0125, 0.023]))}",
        f"imu_rpy:={_as_xyz(cfg.get('imu_rpy', [0, 0, -1.5707963]))}",
        f"mag_xyz:={_as_xyz(cfg.get('mag_xyz', [0.006176, 0.0125, 0.023]))}",
        f"mag_rpy:={_as_xyz(cfg.get('mag_rpy', [-3.1415926, 0, -1.5707963]))}",
        f"body_xyz:={_as_xyz(cfg.get('fast_lio_body_xyz', [0.018482107, -0.012812206, 0.12667]))}",
        f"body_rpy:={_as_xyz(cfg.get('fast_lio_body_rpy', [0, 0, 1.16937]))}",
        f"lidar_xyz:={_as_xyz(cfg.get('lidar_xyz', [0.0, 0.0, 0.25]))}",
        f"lidar_rpy:={_as_xyz(cfg.get('lidar_rpy', [0.0, 0.0, 0.0]))}",
        f"camera_xyz:={_as_xyz(cfg.get('camera_xyz', [0.0, 0.0, 0.20]))}",
        f"camera_rpy:={_as_xyz(cfg.get('camera_rpy', [0.0, 0.0, 0.0]))}",
    ]

    robot_description = subprocess.check_output(xacro_cmd, text=True)

    nodes = []
    if use_joint_state_gui:
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": use_sim_time},
                ],
            )
        )
    else:
        nodes.append(
            Node(
                package="fusionlynx_description",
                executable="static_joint_state_publisher.py",
                name="encoder_joint_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {
                        "joint_names": [
                            "base_link_to_wheel_fl_link",
                            "base_link_to_wheel_fr_link",
                            "base_link_to_wheel_rl_link",
                            "base_link_to_wheel_rr_link",
                        ],
                        "encoder_topic": "/motioncore/encoder_raw",
                        "frame_id": cfg.get("base_link", "base_link"),
                        "publish_rate": float(cfg.get("joint_state_publish_rate", 30.0)),
                        "ticks_per_rev": int(cfg.get("encoder_ticks_per_rev", 2464)),
                        "encoder_sign": cfg.get("encoder_sign", [1, 1, 1, 1]),
                        "zero_offset_ticks": cfg.get("zero_offset_ticks", [0, 0, 0, 0]),
                    },
                ],
            )
        )

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_link_to_unilidar_lidar_tf",
            output="screen",
            arguments=[
                "0", "0", "0",
                "0", "0", "0", "1",
                "lidar_link",
                "unilidar_lidar",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("fusionlynx_description")
    default_params = os.path.join(pkg_share, "config", "robot_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to FusionLynx robot parameter file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with the FusionLynx display config",
        ),
        DeclareLaunchArgument(
            "use_joint_state_gui",
            default_value="false",
            description="Launch joint_state_publisher_gui for manual wheel sliders",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
