import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# -----------------------------------------------------------------------------
# | 比特位 |           功能           |     取值0     |           取值1          |
# |-------|--------------------------|--------------|-------------------------|
# |   0   | 切换标准FOV和广角FOV       | 标准FOV(180°) | 广角FOV(192°)            |
# |   1   | 切换3D和2D测量模式         | 3D测量模式    | 2D测量模式                |
# |   2   | 使能或关闭IMU             | 使能IMU       | 关闭IMU                  |
# |   3   | 切换网口模式和串口模式      | 网口模式       | 串口模式                 |
# |   4   | 切换激光雷达上电默认启动模式 | 上电即自行启动  | 上电保持不转动并等待启动命令 |
# | 5-31  | 保留                     | 保留          | 保留                     |
# -----------------------------------------------------------------------------

def generate_launch_description():
    publish_tf = LaunchConfiguration('publish_tf')

    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                
                {'initialize_type': 2},             # 1: 初始化串口，2: 初始化网口
                {'work_mode': 0},                   # 8: 串口模式，0: 网口模式
                {'use_system_timestamp': True},
                # Modified: 2026-08-20, WenSheng Xu.
                # FusionLynx publishes the canonical lidar_link ->
                # unilidar_lidar identity transform.  Disable the driver's
                # diagnostic TF chain to avoid a second parent for the same
                # physical lidar frame.  Set true only for standalone driver
                # visualization without fusionlynx_description.
                {'publish_tf': publish_tf},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6201},
                {'local_ip': '192.168.1.2'},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    # Run Rviz
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    print("rviz_config_file = " + rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description=(
                'Publish the standalone UniLidar diagnostic TF chain. '
                'Keep false when fusionlynx_description is running.'
            ),
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz2 with the UniLidar view."),
        node1,
        rviz_node,
    ])
