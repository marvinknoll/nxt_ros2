import unittest
from unittest.mock import MagicMock

import rclpy
import rclpy.time
import rclpy.clock

from nxt_ros2.nxt_ros import Motor
import nxt.motor

import sensor_msgs.msg


class TestMotor(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.brick = MagicMock()
        self.name = "test_motor"
        self.invert_direction = False
        self.motor = Motor(
            self.brick, self.name, nxt.motor.Port.A, self.invert_direction
        )

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def test_publisher(self):
        publishers = self.motor.get_publisher_names_and_types_by_node(
            self.motor.get_name(), ""
        )
        joint_state_topic = ("/joint_state", ["sensor_msgs/msg/JointState"])
        self.assertIn(joint_state_topic, publishers)

    def test_subscription(self):
        subscribers = self.motor.get_subscriber_names_and_types_by_node(
            self.motor.get_name(), ""
        )
        joint_effort_subscriber = (
            "/joint_effort",
            ["nxt_msgs2/msg/JointEffort"],
        )
        self.assertIn(joint_effort_subscriber, subscribers)

    def test_motor_turn_action_server_exists(self):
        services = self.motor.get_service_names_and_types()

        turn_action_server = [
            (
                "/test_motor_turn/_action/cancel_goal",
                ["action_msgs/srv/CancelGoal"],
            ),
            (
                "/test_motor_turn/_action/get_result",
                ["nxt_msgs2/action/TurnMotor_GetResult"],
            ),
            (
                "/test_motor_turn/_action/send_goal",
                ["nxt_msgs2/action/TurnMotor_SendGoal"],
            ),
        ]

        for action in turn_action_server:
            self.assertIn(action, services)

    def test_publishes_correct_joint_state(self):
        POWER_TO_NM = 0.01
        motor_positions = [0.0, 6.283185]
        joint_effort = 50.0
        joint_state_effort = (
            joint_effort * POWER_TO_NM * (-1 if self.invert_direction else 1)
        )

        self.motor._effort = joint_effort  # set motor internal effort

        # mock _get_motor_position
        mock_motor_position = MagicMock()
        mock_motor_position.side_effect = motor_positions
        self.motor._get_motor_position = mock_motor_position

        # mock motor node time
        mock_clock = MagicMock()
        mock_clock.side_effect = [
            rclpy.time.Time(
                seconds=1, clock_type=rclpy.clock.ClockType.ROS_TIME
            ),
            rclpy.time.Time(
                seconds=2, clock_type=rclpy.clock.ClockType.ROS_TIME
            ),
        ]
        self.motor.get_clock().now = mock_clock

        # create joint_state subscriber
        mock_cb_run_motor = MagicMock()
        subscriber = rclpy.create_node("subscriber")
        subscriber.create_subscription(
            sensor_msgs.msg.JointState,
            "joint_state",
            mock_cb_run_motor,
            10,
        )

        # assert initial joint_state (no _last_js)
        rclpy.spin_once(self.motor)
        rclpy.spin_once(subscriber)

        expected_initial_js = sensor_msgs.msg.JointState()
        expected_initial_js.header.stamp = mock_cb_run_motor.call_args.args[
            0
        ].header.stamp
        expected_initial_js.effort = [joint_state_effort]
        expected_initial_js.name = [self.motor.get_name()]
        expected_initial_js.position = [motor_positions[0]]
        expected_initial_js.velocity = [0.0]

        mock_cb_run_motor.assert_called_with(expected_initial_js)

        # following joint_state (with _last_js)
        rclpy.spin_once(self.motor)
        rclpy.spin_once(subscriber)

        expected_following_js = sensor_msgs.msg.JointState()
        expected_following_js.header.stamp = mock_cb_run_motor.call_args.args[
            0
        ].header.stamp
        expected_following_js.effort = [joint_state_effort]
        expected_following_js.name = [self.motor.get_name()]
        expected_following_js.position = [motor_positions[1]]
        expected_following_js.velocity = [6.283185]

        mock_cb_run_motor.assert_called_with(expected_following_js)

    def test_not_running_motor_while_turning(self):
        self.motor._turning_lock.acquire()  # Turn action locks this lock
        mock_run = MagicMock()
        self.motor._motor.run = mock_run

        rclpy.spin_once(self.motor)

        mock_run.assert_not_called()

    def test_publishes_js_while_turning(self):
        self.motor._turning_lock.acquire()
        mock_pub = MagicMock()
        self.motor._js_publisher.publish = mock_pub

        rclpy.spin_once(self.motor)

        mock_pub.assert_called_once()
