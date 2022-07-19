import unittest
import unittest.mock

import rclpy

from nxt_ros2.nxt_ros import TouchSensor
import nxt.sensor


class TestTouchSensor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.brick = unittest.mock.Mock()
        self.name = "touch_sensor"
        self.touch_sensor = TouchSensor(
            self.brick, self.name, nxt.sensor.Port.S1
        )

    def tearDown(self) -> None:
        self.touch_sensor.destroy_node()

    def test_publisher(self):
        node_topics = self.touch_sensor.get_topic_names_and_types()
        touch_sensor_topic = ("/" + self.name, ["nxt_msgs2/msg/Touch"])
        self.assertIn(touch_sensor_topic, node_topics)


if __name__ == "__main__":
    unittest.main()
