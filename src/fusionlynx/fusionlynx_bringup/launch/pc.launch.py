"""Launch the FusionLynx PC-side model, fusion, and mapping stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch the FusionLynx RViz model view.",
    )
    use_joint_state_gui = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="false",
        description="Use manual wheel sliders instead of encoder joint states.",
    )
    run_fast_lio = DeclareLaunchArgument(
        "run_fast_lio",
        default_value="true",
        description="Launch the FAST-LIO frontend.",
    )
    run_state_fusion = DeclareLaunchArgument(
        "run_state_fusion",
        default_value="true",
        description="Launch the state fusion backend.",
    )
    run_mapping = DeclareLaunchArgument(
        "run_mapping",
        default_value="true",
        description="Launch the RTAB-Map backend.",
    )
    run_loop_closure = DeclareLaunchArgument(
        "run_loop_closure",
        default_value="false",
        description="Launch the reserved loop-closure interface.",
    )
    fusion_profile = DeclareLaunchArgument(
        "fusion_profile",
        default_value="",
        description="A-G Vault fusion profile, or default for YAML switches.",
    )
    wheel_odom_topic = DeclareLaunchArgument(
        "wheel_odom_topic",
        default_value="",
        description="Wheel odometry observation topic.",
    )
    fast_lio_odom_topic = DeclareLaunchArgument(
        "fast_lio_odom_topic",
        default_value="",
        description="FAST-LIO odometry observation topic.",
    )
    mapping_database_path = DeclareLaunchArgument(
        "mapping_database_path",
        default_value="~/.ros/fusionlynx_rtabmap.db",
        description="RTAB-Map database path.",
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fusionlynx_description"),
                    "launch",
                    "display.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_gui": LaunchConfiguration("use_joint_state_gui"),
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fusionlynx_slam"),
                    "launch",
                    "slam.launch.py",
                ]
            )
        ),
        launch_arguments={
            "run_fast_lio": LaunchConfiguration("run_fast_lio"),
            "run_state_fusion": LaunchConfiguration("run_state_fusion"),
            "run_mapping": LaunchConfiguration("run_mapping"),
            "run_loop_closure": LaunchConfiguration("run_loop_closure"),
            "mapping_database_path": LaunchConfiguration("mapping_database_path"),
            "fusion_profile": LaunchConfiguration("fusion_profile"),
            "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
            "fast_lio_odom_topic": LaunchConfiguration("fast_lio_odom_topic"),
            "fast_lio_rviz": "false",
        }.items(),
    )

    # Give robot_state_publisher and the static sensor transforms a short
    # head start before FAST-LIO, fusion, and RTAB-Map begin consuming data.
    delayed_slam_launch = TimerAction(period=1.0, actions=[slam_launch])

    return LaunchDescription(
        [
            use_rviz,
            use_joint_state_gui,
            run_fast_lio,
            run_state_fusion,
            run_mapping,
        run_loop_closure,
        fusion_profile,
        wheel_odom_topic,
        fast_lio_odom_topic,
        mapping_database_path,
            description_launch,
            delayed_slam_launch,
        ]
    )
