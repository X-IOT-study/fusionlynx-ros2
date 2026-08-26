import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Modification: 2026-08-21, WenSheng Xu.
    # Reason: Provide one reproducible system entry point while keeping
    # FAST-LIO, fusion, mapping, and loop closure as separate packages.
    system_share = get_package_share_directory("fusionlynx_slam")
    default_system_params = os.path.join(
        system_share, "config", "fusionlynx_slam.yaml"
    )
    default_fast_lio_rviz = PathJoinSubstitution(
        [FindPackageShare("fast_lio"), "rviz", "fastlio.rviz"]
    )
    default_fast_lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fast_lio"), "config"]
    )

    run_fast_lio = DeclareLaunchArgument(
        "run_fast_lio",
        default_value="true",
        description="Start the FAST-LIO odometry frontend",
    )
    run_state_fusion = DeclareLaunchArgument(
        "run_state_fusion",
        default_value="true",
        description="Start the state_fusion_backend factor graph",
    )
    run_mapping = DeclareLaunchArgument(
        "run_mapping",
        default_value="true",
        description="Start the RTAB-Map backend through fusionlynx_mapping",
    )
    run_loop_closure = DeclareLaunchArgument(
        "run_loop_closure",
        default_value="false",
        description="Start the reserved loop-closure interface package",
    )
    system_params = DeclareLaunchArgument(
        "system_params_file",
        default_value=default_system_params,
        description="System profile passed to fusion and mapping nodes",
    )
    fusion_profile = DeclareLaunchArgument(
        "fusion_profile",
        default_value="",
        description="A-G Vault fusion profile, or default for YAML switches",
    )
    wheel_odom_topic = DeclareLaunchArgument(
        "wheel_odom_topic",
        default_value="",
        description="Wheel odometry observation topic",
    )
    fast_lio_odom_topic = DeclareLaunchArgument(
        "fast_lio_odom_topic",
        default_value="",
        description="FAST-LIO odometry observation topic",
    )
    mapping_database_path = DeclareLaunchArgument(
        "mapping_database_path",
        default_value="~/.ros/fusionlynx_rtabmap.db",
        description="RTAB-Map database path",
    )
    fast_lio_config_path = DeclareLaunchArgument(
        "fast_lio_config_path",
        default_value=default_fast_lio_config_path,
        description="FAST-LIO parameter directory",
    )
    fast_lio_config_file = DeclareLaunchArgument(
        "fast_lio_config_file",
        default_value="unilidar_L2.yaml",
        description="FAST-LIO parameter filename",
    )
    fast_lio_rviz = DeclareLaunchArgument(
        "fast_lio_rviz",
        default_value="false",
        description="Start FAST-LIO's standalone RViz instance",
    )
    fast_lio_rviz_config = DeclareLaunchArgument(
        "fast_lio_rviz_config",
        default_value=default_fast_lio_rviz,
        description="FAST-LIO RViz configuration",
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("run_fast_lio")),
        launch_arguments={
            "config_path": LaunchConfiguration("fast_lio_config_path"),
            "config_file": LaunchConfiguration("fast_lio_config_file"),
            "rviz": LaunchConfiguration("fast_lio_rviz"),
            "rviz_cfg": LaunchConfiguration("fast_lio_rviz_config"),
            # FusionLynx revision 2026-08-21, WenSheng Xu:
            # state_fusion_backend is the only dynamic odom TF authority.
            "publish_tf": "false",
        }.items(),
    )

    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("state_fusion_backend"),
                    "launch",
                    "fusion.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("run_state_fusion")),
        launch_arguments={
            "params_file": LaunchConfiguration("system_params_file"),
            "fusion_profile": LaunchConfiguration("fusion_profile"),
            "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
            "fast_lio_odom_topic": LaunchConfiguration("fast_lio_odom_topic"),
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fusionlynx_mapping"),
                    "launch",
                    "mapping.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("run_mapping")),
        launch_arguments={
            "database_path": LaunchConfiguration("mapping_database_path"),
        }.items(),
    )

    loop_closure_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fusionlynx_loop_closure"),
                    "launch",
                    "loop_closure.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("run_loop_closure")),
    )

    return LaunchDescription(
        [
            run_fast_lio,
            run_state_fusion,
            run_mapping,
            run_loop_closure,
            system_params,
            fusion_profile,
            wheel_odom_topic,
            fast_lio_odom_topic,
            mapping_database_path,
            fast_lio_config_path,
            fast_lio_config_file,
            fast_lio_rviz,
            fast_lio_rviz_config,
            fast_lio_launch,
            fusion_launch,
            mapping_launch,
            loop_closure_launch,
        ]
    )
