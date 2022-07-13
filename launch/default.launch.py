import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    base_robot_devices_config_file = os.path.join(
        get_package_share_directory('nxt_ros2'),
        'config',
        'base_robot.yaml'
    )

    base_robot_model_path = os.path.join(
        get_package_share_directory('nxt_ros2'), 'urdf', 'base_robot.urdf.xacro')

    base_robot_rviz_config_path = os.path.join(
        get_package_share_directory('nxt_ros2'), 'rviz', 'base_robot.rviz')

    visualize_arg = DeclareLaunchArgument(
        name='visualize', default_value='false', choices=['true', 'false'])

    # Use custom config file with: ros2 launch nxt_ros2 default.launch.py device_config_file:='absolute/path/to/config/file.yaml'
    devices_config_arg = DeclareLaunchArgument(
        name="device_config_file",
        default_value=base_robot_devices_config_file,
        description="Absolute path to config file (.yaml)"
    )

    # Use custom model file with: ros2 launch nxt_ros2 default.launch.py model:='absolute/path/to/config/file.urdf.xacro'
    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(base_robot_model_path),
                                      description='Absolute path to .urdf / .urdf.xacro file')

    # Use custom rviz config file with: ros2 launch nxt_ros2 default.launch.py rviz_config:='absolute/path/to/config/file.rviz'
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config',
                                            default_value=str(
                                                base_robot_rviz_config_path),
                                            description='Absolute path to .rviz file')

    nxt_node = Node(package='nxt_ros2',
                    executable='nxt_ros',
                    parameters=[LaunchConfiguration('device_config_file')]
                    )

    joint_state_aggregator_node = Node(package='nxt_ros2',
                                       executable='js_aggregator',
                                       parameters=[LaunchConfiguration('device_config_file')])

    odometry_node = Node(package='nxt_ros2', executable='odometry')

    differential_drive_controller_node = Node(package='nxt_ros2',
                                              executable='diff_drive_controller',
                                              parameters=[LaunchConfiguration('device_config_file')])

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    return LaunchDescription([
        visualize_arg,
        devices_config_arg,
        model_arg,
        rviz_config_arg,
        nxt_node,
        joint_state_aggregator_node,
        odometry_node,
        differential_drive_controller_node,
        robot_state_publisher_node,
        rviz_node
    ])
