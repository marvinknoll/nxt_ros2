import rclpy
import rclpy.executors
import rclpy.node

import nxt_msgs2.msg
import sensor_msgs.msg
import std_msgs.msg

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

    def destroy(self):
        return super().destroy_node()


class UltraSonicSensor(rclpy.node.Node):
    def __init__(self, comm: nxt.brick.Brick, port: nxt.sensor.Port):
        super().__init__('ultrasonic_sensor')

        self.declare_parameter('field_of_view', 0.5235988)  # 30 degrees
        self.declare_parameter('min_range', 0.07)  # meters
        self.declare_parameter('max_range', 2.54)  # meters

        self._sensor = comm.get_sensor(port, nxt.sensor.generic.Ultrasonic)

        self._publisher = self.create_publisher(
            sensor_msgs.msg.Range, 'ultrasonic_sensor', 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        field_of_view = self.get_parameter(
            'field_of_view').get_parameter_value().double_value
        min_range = self.get_parameter(
            'min_range').get_parameter_value().double_value
        max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value

        msg = sensor_msgs.msg.Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = 0  # ultrasound
        msg.field_of_view = field_of_view
        msg.min_range = min_range
        msg.max_range = max_range
        msg.range = self._sensor.get_sample() / 100  # meters

        self._publisher.publish(msg)

    def destroy(self):
        return super().destroy_node()


class ColorSensor(rclpy.node.Node):
    def __init__(self, comm: nxt.brick.Brick, port: nxt.sensor.Port):
        super().__init__('color_sensor')

        self._sensor = comm.get_sensor(port, nxt.sensor.generic.Color)

        self._publisher = self.create_publisher(
            nxt_msgs2.msg.Color, 'color_sensor', 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        msg = nxt_msgs2.msg.Color()
        msg.header.stamp = self.get_clock().now().to_msg()
        sample = self._sensor.get_color()
        msg.color = self.color_code_to_rgba(sample)

        self._publisher.publish(msg)

    def destroy(self):
        self._sensor.set_light_color(nxt.sensor.Type.COLOR_EXIT)
        return super().destroy_node()

    def color_code_to_rgba(self, color_code: int) -> std_msgs.msg.ColorRGBA:
        """Converts nxt_python's color code to std_msgs.msg.ColorRGBA"""
        color = std_msgs.msg.ColorRGBA()
        if color_code == 1:  # black
            color.r = 0.0
            color.g = 0.0
            color.b = 0.0
        elif color_code == 2:  # blue
            color.r = 0.0
            color.g = 0.0
            color.b = 255.0
        elif color_code == 3:  # green
            color.r = 0.0
            color.g = 255.0
            color.b = 0.0
        elif color_code == 4:  # yellow
            color.r = 255.0
            color.g = 255.0
            color.b = 0.0
        elif color_code == 5:  # red
            color.r = 255.0
            color.g = 0.0
            color.b = 0.0
        elif color_code == 6:  # white
            color.r = 255.0
            color.g = 255.0
            color.b = 255.0
        color.a = 1.0
        return color


def main(args=None):
    rclpy.init(args=args)

    try:
        with nxt.locator.find() as b:
            touch_sensor = TouchSensor(b, nxt.sensor.Port.S1)
            ultrasonic_sensor = UltraSonicSensor(b, nxt.sensor.Port.S4)
            color_sensor = ColorSensor(b, nxt.sensor.Port.S3)

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(touch_sensor)
            executor.add_node(ultrasonic_sensor)
            executor.add_node(color_sensor)

            try:
                executor.spin()
            finally:
                executor.shutdown()

                touch_sensor.destroy()
                ultrasonic_sensor.destroy()
                color_sensor.destroy()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
