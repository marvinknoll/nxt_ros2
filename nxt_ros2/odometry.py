import rclpy
import rclpy.node
import rclpy.executors

from nxt_ros2.util.helper_classes import MotorConfigs, RobotDimensions

import nxt_msgs2.srv
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

import tf2_ros

from typing import Dict
import math


class Odometry(rclpy.node.Node):
    def __init__(self):
        super().__init__("odometry")

        # Get motor_configs from setup node
        self._get_motors_configs_client = self.create_client(
            nxt_msgs2.srv.MotorConfigs, "/nxt_ros_setup/get_motor_configs"
        )
        while not self._get_motors_configs_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "nxt_ros_setup/get_motor_configs service not available,"
                " waiting again..."
            )
        self._motor_configs = self._get_motor_configs()

        if (
            "wheel_motor_r" not in self._motor_configs.motor_types
            or "wheel_motor_l" not in self._motor_configs.motor_types
        ):
            self.get_logger().info(
                "No 'wheel_motor_r' and 'wheel_motor_l' defined in config"
                " params. Stopping odometry node"
            )
            return

        # Get robot dimensions from setup node
        self._get_robot_dimensions_client = self.create_client(
            nxt_msgs2.srv.RobotDimensions,
            "/nxt_ros_setup/get_robot_dimensions",
        )
        while not self._get_robot_dimensions_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "nxt_ros_setup/get_robot_dimensions service not available,"
                " waiting again..."
            )
        self._robot_dimensions: RobotDimensions = self._get_robot_dimensions()

        if (
            self._robot_dimensions.axle_track == -1.0
            or self._robot_dimensions.rad_per_s_to_effort == -1.0
            or self._robot_dimensions.wheel_radius == -1.0
        ):
            self.get_logger().info(
                "No valid 'robot_dimensions' config params. Stopping odometry"
                " node"
            )
            return

        self.last_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_phi = 0.0
        self._last_r_mimic_pos = 0.0
        self._last_l_mimic_pos = 0.0

        self._joint_states_subscriber = self.create_subscription(
            sensor_msgs.msg.JointState,
            "joint_states",
            self._cb_publish_odometry,
            10,
        )
        self._odom_publisher = self.create_publisher(
            nav_msgs.msg.Odometry, "odom", 10
        )
        self._transform_broadcaster = tf2_ros.TransformBroadcaster(self)

    def _get_motor_configs(self) -> MotorConfigs:
        self._req = nxt_msgs2.srv.MotorConfigs.Request()
        future = self._get_motors_configs_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        motor_configs = MotorConfigs()
        motor_configs.motor_ports = result.motor_ports
        motor_configs.motor_types = result.motor_types
        motor_configs.motor_names = result.motor_names
        motor_configs.motor_mimic_names = result.motor_mimic_names
        motor_configs.motor_mimic_gear_ratios = result.motor_mimic_gear_ratios
        motor_configs.invert_directions = result.invert_directions

        if (
            "wheel_motor_r" in motor_configs.motor_types
            and "wheel_motor_l" in motor_configs.motor_types
        ):
            self.get_logger().info(
                "Got valid motor configs from nxt_ros_setup node"
            )

        return motor_configs

    def _get_robot_dimensions(self) -> RobotDimensions:
        self._req = nxt_msgs2.srv.RobotDimensions.Request()
        future = self._get_robot_dimensions_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        robot_dimensions = RobotDimensions()
        robot_dimensions.axle_track = result.axle_track
        robot_dimensions.rad_per_s_to_effort = result.rad_per_s_to_effort
        robot_dimensions.wheel_radius = result.wheel_radius

        if (
            robot_dimensions.axle_track != -1.0
            or robot_dimensions.rad_per_s_to_effort != -1.0
            or robot_dimensions.wheel_radius != -1.0
        ):
            self.get_logger().info(
                "Got valid robot dimensions from nxt_ros_setup node"
            )

        return robot_dimensions

    def _cb_publish_odometry(self, msg: sensor_msgs.msg.JointState):
        r_mimic_index = self._motor_configs.motor_types.index("wheel_motor_r")
        l_mimic_index = self._motor_configs.motor_types.index("wheel_motor_l")

        r_mimic_name = self._motor_configs.motor_mimic_names[r_mimic_index]
        l_mimic_name = self._motor_configs.motor_mimic_names[l_mimic_index]

        if r_mimic_name not in msg.name or l_mimic_name not in msg.name:
            self.get_logger().error(
                "Impossible to calculate odometry without '%s' and '%s' in"
                " JointState." % (r_mimic_name, l_mimic_name)
            )
            return

        r_mimic_pos = msg.position[msg.name.index(r_mimic_name)]
        l_mimic_pos = msg.position[msg.name.index(l_mimic_name)]

        r_mimic_vel = msg.velocity[msg.name.index(r_mimic_name)]
        l_mimic_vel = msg.velocity[msg.name.index(l_mimic_name)]

        rot = (
            (r_mimic_vel - l_mimic_vel) * self._robot_dimensions.wheel_radius
        ) / self._robot_dimensions.axle_track  # rad/s
        trans = (
            r_mimic_vel * 2 * self._robot_dimensions.wheel_radius
            - rot * self._robot_dimensions.axle_track
        ) / 2  # m/s

        delta_r_mimic = (
            r_mimic_pos - self._last_r_mimic_pos
        ) * self._robot_dimensions.wheel_radius
        delta_l_mimic = (
            l_mimic_pos - self._last_l_mimic_pos
        ) * self._robot_dimensions.wheel_radius
        delta_center = (delta_r_mimic + delta_l_mimic) / 2

        phi = self.last_phi + (
            (delta_r_mimic - delta_l_mimic) / self._robot_dimensions.axle_track
        )  # rad
        x = self.last_x + delta_center * math.cos(phi)
        y = self.last_y + delta_center * math.sin(phi)

        quaternion = self.euler_to_quaternion(phi, 0, 0)
        now = self.get_clock().now()

        # Transformation
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self._transform_broadcaster.sendTransform(t)

        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"

        # Odometry Message
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        # TODO odom.pose.covariance

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = trans
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = rot
        # TODO odom.twist.covariance

        self._odom_publisher.publish(odom)

        self.last_time = now
        self._last_r_mimic_pos = r_mimic_pos
        self._last_l_mimic_pos = l_mimic_pos
        self.last_phi = phi
        self.last_x = x
        self.last_y = y

    def euler_to_quaternion(self, yaw, pitch, roll):
        # yaw (Z), pitch (Y), roll (X) in radians
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return [qx, qy, qz, qw]


def main(args=None):
    try:
        rclpy.init(args=args)
        odometry = Odometry()
        rclpy.spin(odometry)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Got clean shutdown signal, shutting down node.")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
