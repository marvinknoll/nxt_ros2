import unittest
from unittest.mock import MagicMock

import rclpy

from nxt_ros2.nxt_ros import TouchSensor
import nxt.sensor
import usb.core

import nxt_msgs2.msg


class TestTouchSensor(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.brick = MagicMock()
        self.name = "touch_sensor"
        self.frame_id = "touch_frame"
        self.touch_sensor = TouchSensor(
            self.brick, self.name, nxt.sensor.Port.S1, self.frame_id
        )

    def tearDown(self):
        self.touch_sensor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_publisher(self):
        publishers = self.touch_sensor.get_publisher_names_and_types_by_node(
            self.touch_sensor.get_name(), ""
        )
        touch_sensor_topic = ("/" + self.name, ["nxt_msgs2/msg/Touch"])
        self.assertIn(touch_sensor_topic, publishers)

    def test_shutdown_on_lost_connection(self):
        throw_USB_error_mock = MagicMock(
            side_effect=usb.core.USBError(
                "No such device (it may have been disconnected)"
            )
        )
        self.touch_sensor._sensor.get_sample = throw_USB_error_mock
        rclpy.spin_once(self.touch_sensor)

        self.assertFalse(rclpy.ok())

    def test_measure_timer_called(self):
        mock_get_sample = MagicMock(return_value=True)
        self.touch_sensor._sensor.get_sample = mock_get_sample
        rclpy.spin_once(self.touch_sensor)

        mock_get_sample.assert_called()

    def test_measurement_message(self):
        touch_sample = True

        self.touch_sensor._sensor.get_sample = MagicMock(
            return_value=touch_sample
        )
        mock_cb = MagicMock()

        subscriber = rclpy.create_node("subscriber")
        subscriber.create_subscription(
            nxt_msgs2.msg.Touch,
            self.touch_sensor.get_name(),
            mock_cb,
            10,
        )

        rclpy.spin_once(self.touch_sensor)
        rclpy.spin_once(subscriber)

        expected_msg = nxt_msgs2.msg.Touch()
        expected_msg.header.stamp = mock_cb.call_args.args[0].header.stamp
        expected_msg.header.frame_id = self.frame_id
        expected_msg.touch = touch_sample

        mock_cb.assert_called_once_with(expected_msg)


if __name__ == "__main__":
    unittest.main()
