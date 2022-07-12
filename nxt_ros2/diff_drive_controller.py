import rclpy
import rclpy.node

import geometry_msgs.msg
import sensor_msgs.msg
import nxt_msgs2.msg
import nxt_msgs2.srv

from nxt_ros2.nxt_ros import MotorConfigs, RobotDimensions


class DiffDriveController(rclpy.node.Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        # Motor configs
        self._get_motors_configs_client = self.create_client(
            nxt_msgs2.srv.MotorConfigs, "/nxt_ros_setup/get_available_motor_configs")
        while not self._get_motors_configs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "nxt_ros_setup/get_available_motor_configs service not available, waiting again...")
        self._motor_configs: MotorConfigs = self.get_motor_configs()

        if "wheel_motor_r" not in self._motor_configs.motor_types or "wheel_motor_l" not in self._motor_configs.motor_types:
            self.get_logger().info(
                "No wheel motors are defined. Stopping differential drive controller node")
            return

        # Robot dimensions
        self._get_robot_dimensions_client = self.create_client(
            nxt_msgs2.srv.RobotDimensions, "/nxt_ros_setup/get_robot_dimensions")
        while not self._get_robot_dimensions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "nxt_ros_setup/get_robot_dimensions service not available, waiting again...")
        self._robot_dimensions: RobotDimensions = self.get_robot_dimensions()

        if self._robot_dimensions.axle_track == -1.0 or self._robot_dimensions.rad_per_s_to_effort == -1.0 or self._robot_dimensions.wheel_radius == -1.0:
            self.get_logger().info(
                "No valid robot dimensions config params. Stopping differential drive controller node")
            return

        # Controller
        self._goal_lin_vel = 0
        self._goal_ang_vel = 0

        self._cmd_vel_subscriber = self.create_subscription(
            geometry_msgs.msg.TwistStamped, "cmd_vel", self.cmd_vel_cb, 10)

        self._joint_states_subscriber = self.create_subscription(
            sensor_msgs.msg.JointState, "joint_states", self.joint_states_cb, 10)

        self._joint_effort_publisher = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10)

    def get_motor_configs(self) -> MotorConfigs:
        self._req = nxt_msgs2.srv.MotorConfigs.Request()
        future = self._get_motors_configs_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        motor_configs = MotorConfigs()
        motor_configs.motor_names = result.motor_names
        motor_configs.motor_mimic_gear_ratios = result.motor_mimic_gear_ratios
        motor_configs.motor_types = result.motor_types
        motor_configs.invert_directions = result.invert_directions

        if "wheel_motor_r" in motor_configs.motor_types and "wheel_motor_l" in motor_configs.motor_types:
            self.get_logger().info("Got available motors configurations from nxt_ros_setup node")

        return motor_configs

    def get_robot_dimensions(self) -> RobotDimensions:
        self._req = nxt_msgs2.srv.RobotDimensions.Request()
        future = self._get_robot_dimensions_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        robot_dimensions = RobotDimensions()
        robot_dimensions.axle_track = result.axle_track
        robot_dimensions.rad_per_s_to_effort = result.rad_per_s_to_effort
        robot_dimensions.wheel_radius = result.wheel_radius

        if robot_dimensions.axle_track != -1.0 or robot_dimensions.rad_per_s_to_effort != -1.0 or robot_dimensions.wheel_radius != -1.0:
            self.get_logger().info("Got valid robot dimensions from nxt_ros_setup node")

        return robot_dimensions

    def cmd_vel_cb(self, msg: geometry_msgs.msg.TwistStamped):
        self._goal_lin_vel = msg.twist.linear.x
        self._goal_ang_vel = msg.twist.angular.z

    def joint_states_cb(self, msg: sensor_msgs.msg.JointState):
        motor_r_index = self._motor_configs.motor_types.index("wheel_motor_r")
        motor_l_index = self._motor_configs.motor_types.index("wheel_motor_l")
        motor_r_name = self._motor_configs.motor_names[motor_r_index]
        motor_l_name = self._motor_configs.motor_names[motor_l_index]
        motor_r_gear_ratio = self._motor_configs.motor_mimic_gear_ratios[motor_r_index]
        motor_l_gear_ratio = self._motor_configs.motor_mimic_gear_ratios[motor_l_index]
        invert_direction_r = self._motor_configs.invert_directions[motor_r_index]
        invert_direction_l = self._motor_configs.invert_directions[motor_l_index]

        if motor_r_name not in msg.name or motor_l_name not in msg.name:
            self.get_logger().warning(
                "JointState from /joint_states not containing required joints: ('wheel_motor_r', 'wheel_motor_l')")
            return

        wheel_r_vel = (2 * self._goal_lin_vel + self._goal_ang_vel *
                       self._robot_dimensions.axle_track) / (2 * self._robot_dimensions.wheel_radius)  # rad/s
        wheel_l_vel = (2 * self._goal_lin_vel - self._goal_ang_vel *
                       self._robot_dimensions.axle_track) / (2 * self._robot_dimensions.wheel_radius)  # rad/s

        motor_r_vel = wheel_r_vel / motor_r_gear_ratio  # rad/s
        motor_l_vel = wheel_l_vel / motor_l_gear_ratio  # rad/s

        motor_r_vel = motor_r_vel * -1 if invert_direction_r else motor_r_vel
        motor_l_vel = motor_l_vel * -1 if invert_direction_l else motor_l_vel

        joint_effort_r = nxt_msgs2.msg.JointEffort()
        joint_effort_r.header.stamp = self.get_clock().now().to_msg()
        joint_effort_r.joint_name = motor_r_name
        joint_effort_r.effort = motor_r_vel * self._robot_dimensions.rad_per_s_to_effort

        joint_effort_l = nxt_msgs2.msg.JointEffort()
        joint_effort_l.header.stamp = self.get_clock().now().to_msg()
        joint_effort_l.joint_name = motor_l_name
        joint_effort_l.effort = motor_l_vel * self._robot_dimensions.rad_per_s_to_effort

        self._joint_effort_publisher.publish(joint_effort_r)
        self._joint_effort_publisher.publish(joint_effort_l)


def main(args=None):
    try:
        rclpy.init(args=args)

        controller = DiffDriveController()

        rclpy.spin(controller)

    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
