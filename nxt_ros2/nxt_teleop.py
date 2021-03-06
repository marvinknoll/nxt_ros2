from typing import List, Union
import rclpy
import rclpy.node
import rclpy.executors

import geometry_msgs.msg
import nxt_msgs2.msg
import nxt_msgs2.srv

import sys
import select
import tty
import termios

"""
Controll a NXT robot via the keyboard.

Continuously publish the linear and angular velocity of the robot as
TwistStamped messages on the topic 'cmd_vel' to make it drive.

Continuously publish the effort of the third motor as a JointEffort
messages on the topic 'joint_effort' to turn it in the desired direction.
"""

MOVE_BINDINGS = {
    "q": (1, 1),  # forward & left
    "w": (1, 0),  # forward
    "e": (1, -1),  # forward & right
    "a": (0, 1),  # turn on sopt counter clockwise
    "d": (0, -1),  # turn on spot clockwise
    "y": (-1, -1),  # backward & left
    "x": (-1, 0),  # backward
    "c": (-1, 1),  # backward & right
}

MOVE_VEL_BINDINGS = {
    "i": (1.5, 1),  # increase linear velocity by 1.5
    "j": (1, 0.5),  # decrease angular velocity by 0.5
    "k": (0.5, 1),  # decrease linear velocity by 0.5
    "l": (1, 1.5),  # increase angular velocity by 1.5
}

# Third motor
CONTROL_BINDINGS = {"r": 1, "f": -1}

CONTROL_VEL_BINDINGS = {"u": 0.9, "o": 1.1}

usage_msg = """
Control your NXT robot!
---------------------------
Moving around:  | Turning the third motor
  q    w    e   | r
  a         d   | f
  y    x    c   |

k/i : decrease/increase linear velocity by the factor of 0.5 / 1.5
j/l : decrease/increase angular velocity by the factor of 0.5 / 1.5
u/o : decrease/increase third motor velocity by the factor of 0.9 / 1.1

Not pressing any keys stops all motors.
Pressing multiple keys is not possible.

CTRL-C to quit
"""

error_msg = """
Communication failed. Please restart the entire nxt_ros2 system and this
script.
"""


def limit(input, low, high):
    if input > high:
        return high
    if input < low:
        return low
    return input


def get_settings():
    if sys.platform == "win32":
        return None
    else:
        return termios.tcgetattr(sys.stdin)


def get_key(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.5)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Teleop(rclpy.node.Node):
    """
    ROS2 node for publishing to 'cmd_vel' and 'joint_effort' topics.

    Requests motor_configs from NxtRos2Setup node via service.
    """

    def __init__(self):
        """
        Create publishers and service_clients for teleoperation.

        Create publisher for 'cmd_vel' topic and 'joint_effort'. Get motor
        configs from NxtRos2Setup node via service.
        """
        super().__init__("teleop")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_linear_vel", 0.113),  # m/s
                ("max_angular_vel", 1.693),  # rad/s
                ("max_effort", 100),
            ],
        )

        self._cmd_vel_publisher = self.create_publisher(
            geometry_msgs.msg.TwistStamped, "cmd_vel", 10
        )

        self._third_motor_je_publisher = self.create_publisher(
            nxt_msgs2.msg.JointEffort, "joint_effort", 10
        )

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
        self._third_motor_name: str = self._get_third_motor_name()

    def _get_third_motor_name(self) -> Union[str, None]:
        """Request motor configs and return name of motor with type 'other'."""
        self._req = nxt_msgs2.srv.MotorConfigs.Request()
        future = self._get_motors_configs_client.call_async(self._req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        motor_types: List[str] = result.motor_types
        motor_names: List[str] = result.motor_names

        if "other" in motor_types:
            third_motor_index = motor_types.index("other")
            third_motor_name = motor_names[third_motor_index]
        else:
            third_motor_name = None

        return third_motor_name

    def publish_twist_and_joint_effort(
        self,
        lin: int,
        ang: int,
        third_motor: int,
        lin_vel: float,
        ang_vel: float,
        third_motor_effort: float,
    ):
        """
        Publish TwistStamped and JointEffort messages based on input.

        Attributes
        ----------
        lin : int
            linear velocity multiplier for positive and negative velocities
        ang : int
            angular velocity multiplier for positive and negative velocities
        third_motor int
            third_motor_effort multiplier for positive and negative efforts
        lin_vel : float
            desired absolute linear velocity
        ang_vel : float
            desired absolute angular velocity
        third_motor_effort : float
            desired absolute effort for third motor

        """
        # TwistStamped
        twist = geometry_msgs.msg.TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        twist.twist.linear.x = float(lin * lin_vel)
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = float(ang * ang_vel)

        self._cmd_vel_publisher.publish(twist)

        # Third motor JointEffort
        effort = third_motor * third_motor_effort
        if self._third_motor_name is not None:
            third_motor_je = nxt_msgs2.msg.JointEffort()
            third_motor_je.header.stamp = self.get_clock().now().to_msg()
            third_motor_je.joint_name = self._third_motor_name
            third_motor_je.effort = float(effort)

            self._third_motor_je_publisher.publish(third_motor_je)

        if self._third_motor_name is None and third_motor != 0:
            self.get_logger().warn("There is no third motor defined")
        elif self._third_motor_name is not None:
            print(
                "{}\nCurrent velocities: linear: {}m/s - angular : {}rad/s -"
                " third motor effort: {}".format(
                    usage_msg,
                    round(lin * lin_vel, 3),
                    round(ang * ang_vel, 3),
                    effort,
                )
            )
        else:
            print(
                "{}\nCurrent velocities: linear: {}m/s - angular : {}rad/s"
                .format(
                    usage_msg, round(lin * lin_vel, 3), round(ang * ang_vel, 3)
                )
            )


def main(args=None):
    rclpy.init(args=args)

    # multipliers for direction / stop
    lin = 0
    ang = 0
    third_motor = 0

    # velocities / effort
    lin_vel = 0.065  # m/s
    ang_vel = 0.85  # rad/s
    third_motor_effort = 30  # nxt.motor.run effort

    teleop = Teleop()

    try:
        settings = get_settings()

        print(usage_msg)

        while 1:
            key = get_key(settings)
            if key in MOVE_BINDINGS.keys():
                lin = MOVE_BINDINGS[key][0]
                ang = MOVE_BINDINGS[key][1]
            elif key in MOVE_VEL_BINDINGS.keys():
                max_lin_vel = teleop.get_parameter("max_linear_vel").value
                max_ang_vel = teleop.get_parameter("max_angular_vel").value
                lin_vel = limit(
                    lin_vel * MOVE_VEL_BINDINGS[key][0],
                    -max_lin_vel,
                    max_lin_vel,
                )
                ang_vel = limit(
                    ang_vel * MOVE_VEL_BINDINGS[key][1],
                    -max_ang_vel,
                    max_ang_vel,
                )
            elif key in CONTROL_BINDINGS.keys():
                third_motor = CONTROL_BINDINGS[key]
            elif key in CONTROL_VEL_BINDINGS.keys():
                max_effort = teleop.get_parameter("max_effort").value
                third_motor_effort = limit(
                    third_motor_effort * CONTROL_VEL_BINDINGS[key],
                    -max_effort,
                    max_effort,
                )
            elif key == "\x03":
                break
            else:
                lin = 0
                ang = 0
                third_motor = 0

            teleop.publish_twist_and_joint_effort(
                lin, ang, third_motor, lin_vel, ang_vel, third_motor_effort
            )

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Got clean shutdown signal, shutting down node.")
        pass

    except Exception as _:
        print(error_msg)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
