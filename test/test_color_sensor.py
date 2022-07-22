import unittest
from unittest.mock import MagicMock

import rclpy

from nxt_ros2.nxt_ros import ColorSensor
import nxt.sensor
import std_msgs.msg
import nxt_msgs2.msg
import usb.core


class TestColorSensor(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.brick = MagicMock()
        self.name = "color_sensor"
        self.frame_id = "color_frame"
        self.color_sensor = ColorSensor(
            self.brick, self.name, nxt.sensor.Port.S3, self.frame_id
        )

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def test_publisher(self):
        publishers = self.color_sensor.get_publisher_names_and_types_by_node(
            self.color_sensor.get_name(), ""
        )
        color_sensor_topic = ("/" + self.name, ["nxt_msgs2/msg/Color"])
        self.assertIn(color_sensor_topic, publishers)

    def test_color_code_to_rgba_called(self):
        color_code_black = 1
        mock_get_color = MagicMock(return_value=color_code_black)  # black
        mock_color_code_to_rgba = MagicMock(
            return_value=std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        )
        self.color_sensor._sensor.get_color = mock_get_color
        self.color_sensor.color_code_to_rgba = mock_color_code_to_rgba

        rclpy.spin_once(self.color_sensor)

        mock_get_color.assert_called()
        mock_color_code_to_rgba.assert_called_once_with(color_code_black)

    def test_measurement_message(self):
        color_sample = 1  # black

        self.color_sensor._sensor.get_color = MagicMock(
            return_value=color_sample
        )
        mock_cb = MagicMock()

        subscriber = rclpy.create_node("subscriber")
        subscriber.create_subscription(
            nxt_msgs2.msg.Color,
            self.color_sensor.get_name(),
            mock_cb,
            10,
        )

        rclpy.spin_once(self.color_sensor)
        rclpy.spin_once(subscriber)

        expected_msg = nxt_msgs2.msg.Color()
        expected_msg.header.stamp = mock_cb.call_args.args[0].header.stamp
        expected_msg.header.frame_id = self.frame_id
        expected_msg.color = std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

        mock_cb.assert_called_once_with(expected_msg)

    def test_color_code_to_rgba(self):
        class TestCase:
            name: str
            input: int
            expected: std_msgs.msg.ColorRGBA()

            def __init__(
                self, name: str, input: int, expected: std_msgs.msg.ColorRGBA
            ):
                self.name = name
                self.input = input
                self.expected = expected

        test_cases = [
            TestCase(
                "black", 1, std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            ),
            TestCase(
                "blue", 2, std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
            ),
            TestCase(
                "green",
                3,
                std_msgs.msg.ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0),
            ),
            TestCase(
                "yellow",
                4,
                std_msgs.msg.ColorRGBA(r=255.0, g=255.0, b=0.0, a=1.0),
            ),
            TestCase(
                "red", 5, std_msgs.msg.ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
            ),
            TestCase(
                "white",
                6,
                std_msgs.msg.ColorRGBA(r=255.0, g=255.0, b=255.0, a=1.0),
            ),
        ]

        for case in test_cases:
            actual = self.color_sensor.color_code_to_rgba(case.input)
            self.assertEqual(case.expected, actual)

    def test_color_code_to_rgba_raises_exception(self):
        with self.assertRaises(Exception):
            self.color_sensor.color_code_to_rgba(-1)
        with self.assertRaises(Exception):
            self.color_sensor.color_code_to_rgba(7)
