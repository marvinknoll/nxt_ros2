import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    base_robot_device_config_file = os.path.join(
        get_package_share_directory('nxt_ros2'),
        'config',
        'base_robot.yaml'
    )

    # Custom config file with: ros2 launch nxt_ros2 default.launch.py config_file:='absolute/path/to/config/file.yaml'
    devices_config_file = DeclareLaunchArgument(
        name="device_config_file",
        default_value=base_robot_device_config_file,
        description="Absolute path to config file (.yaml)"
    )

    nxt_node = Node(package='nxt_ros2',
                    executable='nxt_ros',
                    parameters=[LaunchConfiguration('device_config_file')]
                    )

    joint_state_aggregator_node = Node(package='nxt_ros2',
                                       executable='js_aggregator',
                                       parameters=[LaunchConfiguration('device_config_file')])

    differential_drive_controller_node = Node(package='nxt_ros2',
                                              executable='diff_drive_controller',
                                              parameters=[LaunchConfiguration('device_config_file')])

    return LaunchDescription([
        devices_config_file,
        nxt_node,
        joint_state_aggregator_node,
        differential_drive_controller_node
    ])
