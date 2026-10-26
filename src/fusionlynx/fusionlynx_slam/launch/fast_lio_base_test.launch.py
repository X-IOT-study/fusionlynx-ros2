"""Run FAST-LIO and RTAB-Map without the state-fusion backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory("fusionlynx_description")
    slam_share = get_package_share_directory("fusionlynx_slam")
    system_params = os.path.join(
        slam_share, "config", "fusionlynx_slam.yaml"
    )

    use_rviz = DeclareLaunchArgument("use_rviz", default_value="false")
    use_mapping = DeclareLaunchArgument("use_mapping", default_value="true")
    mapping_database_path = DeclareLaunchArgument(
        "mapping_database_path",
        default_value="~/.ros/fusionlynx_fast_lio_base_test.db",
    )
    localization = DeclareLaunchArgument(
        "localization",
        default_value="false",
        description=(
            "Use the existing RTAB-Map database for localization instead "
            "of extending the map."
        ),
    )
    initial_pose = DeclareLaunchArgument(
        "initial_pose",
        default_value="",
        description=(
            "Initial robot pose in the map frame: "
            "'x y z roll pitch yaw' or quaternion form."
        ),
    )
    rtabmap_args = DeclareLaunchArgument(
        "rtabmap_args",
        default_value=(
            "--Reg/Strategy 1 "
            "--RGBD/ProximityBySpace true "
            "--RGBD/ProximityByTime false "
            "--RGBD/NeighborLinkRefining true "
            "--RGBD/ProximityPathMaxNeighbors 1 "
            "--RGBD/ProximityGlobalScanMap true "
            "--Icp/CorrespondenceRatio 0.1"
        ),
        description="RTAB-Map parameters and command-line flags.",
    )
    fast_lio_config_path = DeclareLaunchArgument(
        "fast_lio_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("fast_lio"), "config"]
        ),
    )
    fast_lio_config_file = DeclareLaunchArgument(
        "fast_lio_config_file",
        default_value="unilidar_L2.yaml",
    )
    tf_lookup_timeout_sec = DeclareLaunchArgument(
        "tf_lookup_timeout_sec",
        default_value="0.05",
        description="Timeout for looking up base_link <- lidar_link.",
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share, "launch", "display.launch.py"
            )
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_gui": "false",
        }.items(),
    )
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "config_path": LaunchConfiguration("fast_lio_config_path"),
            "config_file": LaunchConfiguration("fast_lio_config_file"),
            "rviz": "false",
            "publish_tf": "false",
        }.items(),
    )
    bridge = Node(
        package="fusionlynx_slam",
        executable="fast_lio_base_bridge.py",
        name="fast_lio_base_bridge",
        output="screen",
        parameters=[
            {
                "tf_lookup_timeout_sec": LaunchConfiguration(
                    "tf_lookup_timeout_sec"
                ),
            }
        ],
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
        condition=IfCondition(LaunchConfiguration("use_mapping")),
        launch_arguments={
            "database_path": LaunchConfiguration("mapping_database_path"),
            "localization": LaunchConfiguration("localization"),
            "initial_pose": LaunchConfiguration("initial_pose"),
            "rtabmap_args": LaunchConfiguration("rtabmap_args"),
            "odom_topic": "/fast_lio/base_odom",
            "frame_id": "base_link",
            "scan_cloud_topic": "/fast_lio/cloud_registered_body",
            "publish_tf_map": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            use_rviz,
            use_mapping,
            mapping_database_path,
            localization,
            initial_pose,
            rtabmap_args,
            fast_lio_config_path,
            fast_lio_config_file,
            tf_lookup_timeout_sec,
            description_launch,
            fast_lio_launch,
            bridge,
            TimerAction(period=1.0, actions=[mapping_launch]),
        ]
    )
