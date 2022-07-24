import rclpy
import rclpy.executors
import rclpy.node
import rclpy.time
import rclpy.clock

import sensor_msgs.msg
import nxt_msgs2.srv

from nxt_ros2.util.helper_classes import MotorConfigs

from typing import Dict, List

"""
  Create and spinn a node that combines all individual motor states into a
  unified joint_state that describes the state of all motors and their mimic
  joints.

  The configuration of the joint names, mimic names and the gear ratios
  between them gets requested from the NxtRos2Setup via a service.
"""


class JointState:
    def __init__(self, name, header, position, velocity, effort):
        self.name: str = name
        self.header = header
        self.position: float = position
        self.velocity: float = velocity
        self.effort: float = effort


class JointStateAggregator(rclpy.node.Node):
    """
    Aggregate the joint_state messages from each motor into one message.

    Subscribe to the joint_state topic, and once observed one joint_state
    from all joints, combine them into a single JointState message and
    publish it to /joint_states. Additionally, it calculate the position,
    velocity and effort of the according mimic joint for each motor which get
    added to the published message. To calculate the state of the mimic joints
    request the motor configs from /nxt_ros_setup setup node.
    """

    def __init__(self):
        """
        Request config, declare param, create subscriber & publisher.

        Request motor configs from NxtRos2Setup node. Subscribe to
        'joint_states' to aggregate the individual joint_states on incomming
        messages. Create a publisher to publish the aggregated joint_states to.
        """
        super().__init__("joint_state_aggregator")

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

        self._motor_configs: MotorConfigs = self._get_motor_configs()
        self._observed_joint_states: Dict[str, JointState] = {}

        self.declare_parameter("related_js_time_delta", 1000000000)

        self._joint_state_subscriber = self.create_subscription(
            sensor_msgs.msg.JointState,
            "joint_state",
            self._cb_join_joint_state_to_states,
            10,
        )
        self._joint_states_publisher = self.create_publisher(
            sensor_msgs.msg.JointState, "joint_states", 10
        )

    def _get_motor_configs(self) -> MotorConfigs:
        """Request motor configs from NxtRos2Setup node and return it."""
        self._req = nxt_msgs2.srv.MotorConfigs.Request()
        future = self._get_motors_configs_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        motor_configs = MotorConfigs()
        motor_configs.motor_names = result.motor_names
        motor_configs.motor_mimic_names = result.motor_mimic_names
        motor_configs.motor_mimic_gear_ratios = result.motor_mimic_gear_ratios

        self.get_logger().info(
            "Got available motors configurations from nxt_ros_setup node"
        )

        return motor_configs

    def _cb_join_joint_state_to_states(self, msg: sensor_msgs.msg.JointState):
        """
        Aggregate joint_state to joint_states and calculate mimic joints.

        Calculate the mimic joint state for each motor and once recived a
        recent joint_state from all motors publish the state of all motors
        and their mimic joints to the 'joint_state' topic.
        """
        if self._motor_configs is None:
            self.get_logger().info(
                "Motor configs not available yet, waiting to be available."
            )
            return

        self._observed_joint_states[msg.name[0]] = JointState(
            msg.name[0],
            msg.header,
            msg.position[0],
            msg.velocity[0],
            msg.effort[0],
        )

        related_js_time_delta_ns = (
            self.get_parameter("related_js_time_delta")
            .get_parameter_value()
            .integer_value
        )

        # Find and delete joint states older than related_js_time_delta
        js_names_to_delete: List[str] = []
        for k, v in self._observed_joint_states.items():
            observed_stamp = rclpy.time.Time(
                seconds=v.header.stamp.sec,
                nanoseconds=v.header.stamp.nanosec,
                clock_type=rclpy.clock.ClockType.ROS_TIME,
            )
            msg_stamp = rclpy.time.Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
                clock_type=rclpy.clock.ClockType.ROS_TIME,
            )
            delta_t = msg_stamp - observed_stamp
            max_duration_for_related_js = rclpy.time.Duration(
                nanoseconds=related_js_time_delta_ns
            )
            if delta_t > max_duration_for_related_js:
                self.get_logger().warn(
                    "JointState older than related_js_time_delta!"
                )
                js_names_to_delete.append(k)

        for js_name in js_names_to_delete:
            del self._observed_joint_states[js_name]

        # Only publish all motor states together
        for motor_name in self._motor_configs.motor_names:
            if motor_name not in self._observed_joint_states.keys():
                return

        joint_states = sensor_msgs.msg.JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()

        for k, v in self._observed_joint_states.items():
            # motor joint
            joint_states.name.append(v.name)
            joint_states.position.append(v.position)
            joint_states.velocity.append(v.velocity)
            joint_states.effort.append(v.effort)

            # mimic joint
            mimic_index = self._motor_configs.motor_names.index(v.name)
            mimic_name = self._motor_configs.motor_mimic_names[mimic_index]
            motor_mimic_gear_ratio = (
                self._motor_configs.motor_mimic_gear_ratios[mimic_index]
            )

            joint_states.name.append(mimic_name)
            joint_states.position.append(v.position * motor_mimic_gear_ratio)
            joint_states.velocity.append(v.velocity * motor_mimic_gear_ratio)
            joint_states.effort.append(v.effort)

        self._joint_states_publisher.publish(joint_states)


def main(args=None):
    try:
        rclpy.init(args=args)
        js_agg = JointStateAggregator()
        rclpy.spin(js_agg)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Got clean shutdown signal, shutting down node.")

    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
