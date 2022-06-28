import rclpy
import rclpy.executors
import rclpy.node

import nxt_msgs2.msg
from sensor_msgs.msg import Range

import nxt
import nxt.locator
import nxt.brick
import nxt.sensor
import nxt.sensor.generic


class TouchSensor(rclpy.node.Node):
    def __init__(self, comm: nxt.brick.Brick, port: nxt.sensor.Port):
        super().__init__('touch_sensor')

        self._sensor = comm.get_sensor(port, nxt.sensor.generic.Touch)

        self._publisher = self.create_publisher(
            nxt_msgs2.msg.Touch, 'touch_sensor', 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        msg = nxt_msgs2.msg.Touch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.touch = self._sensor.get_sample()
        self._publisher.publish(msg)

    def destroy_node(self):
        super().destroy_node()


class UltraSonicSensor(rclpy.node.Node):
    def __init__(self, comm: nxt.brick.Brick, port: nxt.sensor.Port):
        super().__init__('ultrasonic_sensor')

        self.declare_parameter('field_of_view', 0.5235988)  # 30 degrees
        self.declare_parameter('min_range', 0.07)  # meters
        self.declare_parameter('max_range', 2.54)  # meters

        self._sensor = comm.get_sensor(port, nxt.sensor.generic.Ultrasonic)

        self._publisher = self.create_publisher(
            Range, 'ultrasonic_sensor', 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        field_of_view = self.get_parameter(
            'field_of_view').get_parameter_value().double_value
        min_range = self.get_parameter(
            'min_range').get_parameter_value().double_value
        max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = 0  # ultrasound
        msg.field_of_view = field_of_view
        msg.min_range = min_range
        msg.max_range = max_range
        msg.range = self._sensor.get_sample() / 100  # meters

        self._publisher.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        with nxt.locator.find() as b:
            touch_sensor = TouchSensor(b, nxt.sensor.Port.S1)
            ultrasonic_sensor = UltraSonicSensor(b, nxt.sensor.Port.S4)

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(touch_sensor)
            executor.add_node(ultrasonic_sensor)

            try:
                executor.spin()
            finally:
                executor.shutdown()

                for node in executor._nodes:
                    node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
