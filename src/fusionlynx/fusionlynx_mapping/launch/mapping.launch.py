"""Launch RTAB-Map with the FusionLynx mapping input contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value="~/.ros/fusionlynx_rtabmap.db",
                description="RTAB-Map database path.",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description=(
                    "Load the existing RTAB-Map database without adding "
                    "new map nodes."
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="",
                description=(
                    "Initial pose in the map frame: "
                    "'x y z roll pitch yaw' or quaternion form."
                ),
            ),
            DeclareLaunchArgument(
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
                description=(
                    "RTAB-Map parameters and flags. The default enables "
                    "ICP-based LiDAR proximity loop closure."
                ),
            ),
            DeclareLaunchArgument(
                "approx_sync",
                default_value="true",
                description="Use approximate synchronization for odom and cloud.",
            ),
            DeclareLaunchArgument(
                "approx_sync_max_interval",
                default_value="0.10",
                description="Maximum odom/cloud timestamp difference in seconds.",
            ),
            DeclareLaunchArgument(
                "topic_queue_size",
                default_value="30",
                description="Per-topic RTAB-Map input queue size.",
            ),
            DeclareLaunchArgument(
                "sync_queue_size",
                default_value="30",
                description="RTAB-Map synchronization queue size.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="FusionLynx robot base frame.",
            ),
            DeclareLaunchArgument(
                "map_frame_id",
                default_value="map",
                description="RTAB-Map global frame.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Authoritative fused odometry topic.",
            ),
            DeclareLaunchArgument(
                "scan_cloud_topic",
                default_value="/fast_lio/cloud_registered_body",
                description="Deskewed FAST-LIO body-frame point cloud topic.",
            ),
            DeclareLaunchArgument(
                "publish_tf_map",
                default_value="true",
                description="Publish RTAB-Map map -> odom TF.",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz",
                default_value="false",
                description="Launch the RTAB-Map visualization GUI.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rtabmap_launch"),
                            "launch",
                            "rtabmap.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "frame_id": LaunchConfiguration("frame_id"),
                    "map_frame_id": LaunchConfiguration("map_frame_id"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "publish_tf_odom": "false",
                    "publish_tf_map": LaunchConfiguration("publish_tf_map"),
                    "subscribe_scan_cloud": "true",
                    "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic"),
                    "scan_normal_k": "0",
                    "qos": "2",
                    "depth": "false",
                    "visual_odometry": "false",
                    "icp_odometry": "false",
                    "subscribe_rgb": "false",
                    "subscribe_rgbd": "false",
                    "approx_sync": LaunchConfiguration("approx_sync"),
                    "approx_sync_max_interval": LaunchConfiguration(
                        "approx_sync_max_interval"
                    ),
                    "topic_queue_size": LaunchConfiguration("topic_queue_size"),
                    "sync_queue_size": LaunchConfiguration("sync_queue_size"),
                    "rviz": "false",
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "database_path": LaunchConfiguration("database_path"),
                    "localization": LaunchConfiguration("localization"),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "rtabmap_args": LaunchConfiguration("rtabmap_args"),
                }.items(),
            ),
        ]
    )
