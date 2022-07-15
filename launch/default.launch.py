import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file, yaml.SafeLoader)
    except EnvironmentError as e:
        print(str(e))
        return None


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
                    output='screen',
                    parameters=[LaunchConfiguration('device_config_file')],
                    on_exit=Shutdown()
                    )

    joint_state_aggregator_node = Node(package='nxt_ros2',
                                       executable='js_aggregator',
                                       output='screen',
                                       parameters=[LaunchConfiguration('device_config_file')])

    odometry_node = Node(package='nxt_ros2',
                         output='screen',
                         executable='odometry')

    differential_drive_controller_node = Node(package='nxt_ros2',
                                              executable='diff_drive_controller',
                                              output="screen",
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
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    config = load_yaml_file(base_robot_devices_config_file)
    nxt_ros_devices = config["/nxt_ros_setup"]["ros__parameters"]["devices"]

    valid_motor_ports = ["A", "B", "C"]
    motor_types = []
    for port in valid_motor_ports:
        if port in nxt_ros_devices and "motor_type" in nxt_ros_devices[port]:
            motor_types.append(nxt_ros_devices[port]["motor_type"])

    initial_entities = []
    initial_entities.append(visualize_arg)
    initial_entities.append(devices_config_arg)
    initial_entities.append(model_arg)
    initial_entities.append(rviz_config_arg)
    initial_entities.append(nxt_node)

    # Note: this only works with the base_robot_devices_config_file and not with custom config files!
    if "A" in nxt_ros_devices or "B" in nxt_ros_devices or "C" in nxt_ros_devices:
        initial_entities.append(joint_state_aggregator_node)

    if "wheel_motor_l" in motor_types and "wheel_motor_r" in motor_types:
        initial_entities.append(differential_drive_controller_node)
        initial_entities.append(odometry_node)

    initial_entities.append(robot_state_publisher_node)
    initial_entities.append(rviz_node)

    return LaunchDescription(initial_entities=initial_entities)
