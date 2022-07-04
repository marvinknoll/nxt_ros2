import rclpy
import rclpy.executors
import rclpy.node
import rclpy.time
import rclpy.duration
import rclpy.clock
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
import nxt.motor

from typing import Dict, List, Union
import math


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


class Motor(rclpy.node.Node):
    def __init__(self, brick: nxt.brick.Brick, name: str, port: nxt.motor.Port):
        super().__init__(name)

        self._port = port
        self._motor = brick.get_motor(port)
        self._last_js = None
        self._effort = 0
        self._POWER_TO_NM = 0.01

        self._motor.reset_position(False)

        self._jc_subscriber = self.create_subscription(
            nxt_msgs2.msg.JointEffort, "joint_effort", self.joint_effort_cb, 10)

        self._js_publisher = self.create_publisher(
            sensor_msgs.msg.JointState, "joint_state", 10)

        timer_period = 0.1  # seconds
        self.create_timer(timer_period, self.motor_cb)

    def joint_effort_cb(self, msg: nxt_msgs2.msg.JointEffort):
        if msg.joint_name == self.get_name():
            self._effort = msg.effort

    def motor_cb(self):
        tacho = self._motor.get_tacho()
        now = self.get_clock().now()
        position_rad = math.radians(tacho.rotation_count)
        joint_name = self.get_name()
        joint_effort = self._effort * self._POWER_TO_NM
        velocity = 0

        if self._last_js:
            last_stamp = self._last_js.header.stamp
            last_js_now = rclpy.time.Time(seconds=last_stamp.sec,
                                          nanoseconds=last_stamp.nanosec,
                                          clock_type=rclpy.clock.ClockType.ROS_TIME)

            deltaSeconds = (now - last_js_now).nanoseconds/1000000000
            deltaPosition = position_rad - self._last_js.position[0]

            velocity = (deltaPosition / deltaSeconds)

        js = sensor_msgs.msg.JointState()
        js.header.stamp = now.to_msg()
        js.name.append(joint_name)
        js.effort.append(joint_effort)
        js.position.append(position_rad)
        js.velocity.append(velocity)

        self._js_publisher.publish(js)
        self._last_js = js

        self._motor.run(int(self._effort), True)

    def destroy_node(self):
        self._motor.idle()
        return super().destroy_node()


class NxtRos2Setup(rclpy.node.Node):
    """Helper node to read ros2 parameters required for setting up the device-nodes"""

    def __init__(self, brick: nxt.brick.Brick):
        self._brick = brick

        super().__init__("nxt_ros_setup", allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

    def get_sensor_configs(self) -> List[rclpy.node.Node]:
        sensor_nodes: List[Union[TouchSensor, UltraSonicSensor,
                                 ColorSensor, ReflectedLightSensor]] = []
        for port_int in range(1, 5):  # NXT sensor ports (1-4)
            sensor_params: Dict[str, rclpy.Parameter] = self.get_parameters_by_prefix(
                str(port_int))

            if 'sensor_type' in sensor_params and 'sensor_name' in sensor_params:
                sensor_type = sensor_params['sensor_type'].value
                sensor_node_name = sensor_params['sensor_name'].value
                sensor_node_names = map(
                    lambda node: node.get_name(), sensor_nodes)

                if sensor_node_name not in sensor_node_names:
                    port_enum = self.int_to_sensor_port_enum(port_int)
                    if sensor_type == "touch":
                        sensor_nodes.append(
                            TouchSensor(self._brick, sensor_node_name, port_enum))
                    elif sensor_type == "ultrasonic":
                        sensor_nodes.append(UltraSonicSensor(
                            self._brick, sensor_node_name, port_enum))
                    elif sensor_type == "color":
                        sensor_nodes.append(
                            ColorSensor(self._brick, sensor_node_name, port_enum))
                    elif sensor_type == "reflected_light":
                        sensor_nodes.append(ReflectedLightSensor(
                            self._brick, sensor_node_name, port_enum))
                    else:
                        raise Exception(
                            "Invalid sensor type in config parameters: %s" % sensor_type)

                    self.get_logger().info("Created %s sensor with node name '%s' on port '%s'" %
                                           (sensor_type, sensor_node_name, port_enum))
                else:
                    raise Exception(
                        "Duplicate sensor name in config parameters: %s" % sensor_node_name)

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
            raise Exception("Invalid sensor port in config parameters")

    def get_motor_configs(self) -> List[Motor]:
        motor_nodes: List[Motor] = []

        valid_ports = ["A", "B", "C"]
        valid_motor_types = ['wheel_motor_r', 'wheel_motor_l', 'other']
        required_motor_params = {'motor_joint_name', 'motor_type',
                                 'mimic_joint_name', 'motor_mimic_gear_ratio'}
        used_mimic_joint_names: List[str] = []
        used_motor_types: List[str] = []

        for port_str in valid_ports:
            motor_params: Dict[str, rclpy.Parameter] = self.get_parameters_by_prefix(
                port_str)

            if not motor_params.keys() >= required_motor_params:
                raise Exception(
                    "Missing or invalid motor config parameters for motor '%s', required: %s" % (port_str, required_motor_params))

            motor_type = motor_params['motor_type'].value
            motor_joint_name = motor_params['motor_joint_name'].value
            mimic_joint_name = motor_params['mimic_joint_name'].value

            if motor_type not in valid_motor_types:
                raise Exception(
                    "Invalid motor_type config: '%s' for motor '%s'. Please use one of the following: %s" % (motor_type, port_str, valid_motor_types))

            motor_joint_names = map(lambda node: node.get_name(), motor_nodes)

            if motor_joint_name in motor_joint_names:
                raise Exception(
                    "Duplicate motor_joint_name in config parameters: %s" % motor_joint_name)

            if mimic_joint_name in used_mimic_joint_names:
                raise Exception(
                    "Duplicate mimic_joint_name in config parameters: %s" % mimic_joint_name)

            port_enum = self.string_to_motor_port_enum(
                port_str)

            motor_nodes.append(
                Motor(self._brick, motor_joint_name, port_enum))

            used_mimic_joint_names.append(mimic_joint_name)
            used_motor_types.append(motor_type)

        # If you define the wheel motor for one side, you must also define one for the opposite side for the 2-wheel differential drive controller to work
        if 'wheel_motor_r' in used_motor_types and 'wheel_motor_l' not in used_motor_types:
            raise Exception(
                "If you define a motor with motor_type 'wheel_motor_r', please also define one with motor_type: 'wheel_motor_l'")
        if 'wheel_motor_l' in used_motor_types and 'wheel_motor_r' not in used_motor_types:
            raise Exception(
                "If you define a motor with motor_type 'wheel_motor_l', please also define one with motor_type:'wheel_motor_r'")

        for motor_node in motor_nodes:
            self.get_logger().info("Created motor with node name '%s' on port '%s'" %
                                   (motor_node.get_name(), motor_node._port))

        return motor_nodes

    def string_to_motor_port_enum(self, port: str) -> nxt.motor.Port:
        if port == "A":
            return nxt.motor.Port.A
        elif port == "B":
            return nxt.motor.Port.B
        elif port == "C":
            return nxt.motor.Port.C
        else:
            raise Exception("Invalid motor port in config parameters")

    def check_ports_config_parameters(self) -> bool:
        validPorts: List[Union[str, int]] = [1, 2, 3, 4, "A", "B", "C"]
        params = self.get_parameters_by_prefix("")
        params = map(lambda param: param.split(".")[0], params)
        params = filter(lambda param: param != 'use_sim_time', params)

        for params in params:
            if params not in map(lambda port: str(port), validPorts):
                raise Exception("Invalid port config parameter: %s" % params)


def main(args=None):
    rclpy.init(args=args)

    try:
        with nxt.locator.find() as brick:
            setup_node = NxtRos2Setup(brick)
            executor = rclpy.executors.MultiThreadedExecutor()

            nodes: List[Union[TouchSensor, UltraSonicSensor,
                              ColorSensor, ReflectedLightSensor, Motor]] = []

            try:
                setup_node.check_ports_config_parameters()
                nodes.extend(setup_node.get_sensor_configs())
                nodes.extend(setup_node.get_motor_configs())

                for node in nodes:
                    executor.add_node(node)

                executor.spin()
            finally:
                for node in nodes:
                    node.destroy_node()

                executor.shutdown()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
