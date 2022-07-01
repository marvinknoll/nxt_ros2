import rclpy
import rclpy.executors
import rclpy.node
import rclpy.callback_groups
import rcl_interfaces.msg

import nxt_msgs2.msg
import sensor_msgs.msg
import std_msgs.msg

import nxt
import nxt.locator
import nxt.brick
import nxt.sensor
import nxt.sensor.generic

from typing import Dict, List


class TouchSensor(rclpy.node.Node):
    def __init__(self, brick: nxt.brick.Brick, name: str, port: nxt.sensor.Port):
        super().__init__(name)

        self._sensor = brick.get_sensor(port, nxt.sensor.generic.Touch)

        self._publisher = self.create_publisher(
            nxt_msgs2.msg.Touch, name, 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        msg = nxt_msgs2.msg.Touch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.touch = self._sensor.get_sample()
        self._publisher.publish(msg)

    def destroy_node(self):
        return super().destroy_node()


class UltraSonicSensor(rclpy.node.Node):
    def __init__(self, brick: nxt.brick.Brick, name: str, port: nxt.sensor.Port):
        super().__init__(name)

        # Default values for LEGO Mindstorms NXT ultrasonic sensor
        self.declare_parameters(namespace="", parameters=[
            ('field_of_view', 0.5235988),  # 30 degrees
            ('min_range', 0.07),  # meters
            ('max_range', 2.54)])  # meters

        self._sensor = brick.get_sensor(port, nxt.sensor.generic.Ultrasonic)

        self._publisher = self.create_publisher(
            sensor_msgs.msg.Range, name, 10)

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

    def destroy_node(self):
        return super().destroy_node()


class ColorSensor(rclpy.node.Node):
    def __init__(self, brick: nxt.brick.Brick, name: str, port: nxt.sensor.Port):
        super().__init__(name)

        self._sensor = brick.get_sensor(port, nxt.sensor.generic.Color)

        self._publisher = self.create_publisher(
            nxt_msgs2.msg.Color, name, 10)

        timer_period = 0.3  # seconds
        self._timer = self.create_timer(timer_period, self.measure)

    def measure(self):
        msg = nxt_msgs2.msg.Color()
        msg.header.stamp = self.get_clock().now().to_msg()
        sample = self._sensor.get_color()
        msg.color = self.color_code_to_rgba(sample)

        self._publisher.publish(msg)

    def destroy_node(self):
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


class ReflectedLightSensor(rclpy.node.Node):
    def __init__(self, brick: nxt.brick.Brick, name: str, port: nxt.sensor.Port):
        super().__init__(name)

        self._sensor = brick.get_sensor(port, nxt.sensor.generic.Color)

        self._publisher = self.create_publisher(
            nxt_msgs2.msg.Color, name, 10)

        self.declare_parameter('rgb_color', [0.0, 0.0, 0.0])
        self.add_on_set_parameters_callback(self.set_rgb_color_param)

        timer_period = 0.3
        self._timer = self.create_timer(timer_period, self.measure)

    def set_rgb_color_param(self, params: List[rclpy.Parameter]):
        updated_param = False
        for param in params:
            if param.name == "rgb_color" and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY:
                rgb: List[float] = param.value
                color = self.rgb_to_color_type(rgb)
                param_is_valid = color != nxt.sensor.Type.COLOR_EXIT
                updated_param = param_is_valid

        return rcl_interfaces.msg.SetParametersResult(successful=updated_param)

    def measure(self):
        rgb = self.get_parameter(
            'rgb_color').get_parameter_value().double_array_value
        color = self.rgb_to_color_type(rgb)

        reflected_light = nxt_msgs2.msg.Color()
        reflected_light.header.stamp = self.get_clock().now().to_msg()
        reflected_light.color.a = float(
            self._sensor.get_reflected_light(color))
        reflected_light.color.r = rgb[0]
        reflected_light.color.g = rgb[1]
        reflected_light.color.b = rgb[2]

        self._publisher.publish(reflected_light)

    def rgb_to_color_type(self, rgb: List[float]):
        """Converts List [r: float, g: float, b: float] to nxt_python's color code."""
        if rgb[0] == 1.0 and rgb[1] == 0.0 and rgb[2] == 0.0:
            return nxt.sensor.Type.COLOR_RED
        elif rgb[0] == 0.0 and rgb[1] == 1.0 and rgb[2] == 0.0:
            return nxt.sensor.Type.COLOR_GREEN
        elif rgb[0] == 0.0 and rgb[1] == 0.0 and rgb[2] == 1.0:
            return nxt.sensor.Type.COLOR_BLUE
        elif rgb[0] == 1.0 and rgb[1] == 1.0 and rgb[2] == 1.0:
            return nxt.sensor.Type.COLOR_FULL
        elif rgb[0] == 0.0 and rgb[1] == 0.0 and rgb[2] == 0.0:
            return nxt.sensor.Type.COLOR_NONE
        else:
            return nxt.sensor.Type.COLOR_EXIT

    def destroy_node(self):
        self._sensor.set_light_color(nxt.sensor.Type.COLOR_EXIT)
        return super().destroy_node()


class NxtRos2Setup(rclpy.node.Node):
    """Helper node to read ros2 parameters required for setting up the device-nodes"""

    def __init__(self):
        super().__init__("nxt_ros_setup", allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

    def get_sensor_configs(self, brick: nxt.brick.Brick) -> List[rclpy.node.Node]:
        sensor_nodes: List[rclpy.node.Node] = []
        for port_int in range(1, 5):  # NXT sensor ports (1-4)
            sensor_params: Dict[str, rclpy.Parameter] = self.get_parameters_by_prefix(
                str(port_int))

            if 'type' in sensor_params and 'name' in sensor_params:
                type = sensor_params['type'].value
                name = sensor_params['name'].value

                if name not in map(lambda node: node.get_name(), sensor_nodes):
                    port_enum = self.int_to_sensor_port_enum(port_int)
                    if type == "touch":
                        sensor_nodes.append(
                            TouchSensor(brick, name, port_enum))
                    elif type == "ultrasonic":
                        sensor_nodes.append(UltraSonicSensor(
                            brick, name, port_enum))
                    elif type == "color":
                        sensor_nodes.append(
                            ColorSensor(brick, name, port_enum))
                    elif type == "reflected_light":
                        sensor_nodes.append(ReflectedLightSensor(
                            brick, name, port_enum))
                    else:
                        raise Exception(
                            "Invalid sensor type in config parameters: %s" % type)

                    self.get_logger().info("Created %s sensor with node name %s on port %s" %
                                           (type, name, port_enum))
                else:
                    raise Exception(
                        "Duplicate node name in config parameters: %s" % name)

        return sensor_nodes

    def int_to_sensor_port_enum(self, port: int) -> nxt.sensor.Port:
        if port == 1:
            return nxt.sensor.Port.S1
        elif port == 2:
            return nxt.sensor.Port.S2
        elif port == 3:
            return nxt.sensor.Port.S3
        elif port == 4:
            return nxt.sensor.Port.S4
        else:
            raise Exception("Invalid port in config parameters")


def main(args=None):
    rclpy.init(args=args)

    try:
        with nxt.locator.find() as brick:
            setup_node = NxtRos2Setup()
            sensor_nodes = setup_node.get_sensor_configs(setup_node, brick)
            executor = rclpy.executors.MultiThreadedExecutor()

            for sensor_node in sensor_nodes:
                executor.add_node(sensor_node)

            setup_node.destroy_node()  # Only required for setup

            try:
                executor.spin()
            finally:
                for node in executor.get_nodes():
                    node.destroy_node()

                executor.shutdown()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
