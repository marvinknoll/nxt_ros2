import os
import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.asserts
import launch_testing.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import pytest


@pytest.mark.brick_required
def generate_test_description():
    base_robot_devices_config_file = os.path.join(
        get_package_share_directory("nxt_ros2"), "config", "base_robot.yaml"
    )

    nxt_node = Node(
        package="nxt_ros2",
        executable="nxt_ros",
        output="screen",
        parameters=[base_robot_devices_config_file],
    )

    desc = launch.LaunchDescription(
        [
            nxt_node,
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )
    context = {"nxt_node": nxt_node}

    return desc, context


# Run this tests with the following command: launch_test path/to/this/file
# Note: To run tese tests, you have to build the package and connect a nxt
#       brick
@pytest.mark.brick_required
class TestNxtRos2Setup(unittest.TestCase):
    def test_sensor_nodes_creation(self, proc_output, nxt_node):
        proc_output.assertWaitFor(
            "Created sensor of type 'touch' with node name 'touch_sensor' on"
            " port 'Port.S1'",
            timeout=1,
            process=nxt_node,
        )
        proc_output.assertWaitFor(
            "Created sensor of type 'reflected_light' with node name"
            " 'reflected_light_sensor' on port 'Port.S3'",
            timeout=1,
            process=nxt_node,
        )
        proc_output.assertWaitFor(
            "Created sensor of type 'ultrasonic' with node name"
            " 'ultrasonic_sensor' on port 'Port.S4'",
            timeout=1,
            process=nxt_node,
        )

    def test_motor_nodes_creation(self, proc_output, nxt_node):
        proc_output.assertWaitFor(
            "Created motor of type 'other' with node name 'radar_motor' on"
            " port 'Port.A'",
            timeout=1,
            process=nxt_node,
        )
        proc_output.assertWaitFor(
            "Created motor of type 'wheel_motor_l' with node name"
            " 'wheel_motor_l' on port 'Port.B'",
            timeout=1,
            process=nxt_node,
        )
        proc_output.assertWaitFor(
            "Created motor of type 'wheel_motor_r' with node name"
            " 'wheel_motor_r' on port 'Port.C'",
            timeout=1,
            process=nxt_node,
        )
