import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_fusion_node(context, *args, **kwargs):
    del args, kwargs
    overrides = {}
    fusion_profile = LaunchConfiguration("fusion_profile").perform(context)
    wheel_topic = LaunchConfiguration("wheel_odom_topic").perform(context)
    lio_topic = LaunchConfiguration("fast_lio_odom_topic").perform(context)
    if fusion_profile:
        overrides["fusion_profile"] = fusion_profile
    if wheel_topic:
        overrides["wheel_odom_topic"] = wheel_topic
    if lio_topic:
        overrides["fastlio_topic"] = lio_topic

    return [
        Node(
            package="state_fusion_backend",
            executable="state_fusion_backend_node",
            name="state_fusion_backend_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), overrides],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("state_fusion_backend")
    default_params = os.path.join(package_share, "config", "state_fusion_backend.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Fusion backend parameter file.",
            ),
            DeclareLaunchArgument(
                "fusion_profile",
                default_value="",
                description="A-G Vault profile, or default to use YAML switches.",
            ),
            DeclareLaunchArgument(
                "wheel_odom_topic",
                default_value="",
                description="Wheel odometry observation topic.",
            ),
            DeclareLaunchArgument(
                "fast_lio_odom_topic",
                default_value="",
                description="FAST-LIO odometry observation topic.",
            ),
            OpaqueFunction(function=_create_fusion_node),
        ]
    )
