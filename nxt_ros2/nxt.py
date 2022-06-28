import rclpy
import rclpy.executors
import rclpy.node

import nxt_msgs2.msg

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
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        with nxt.locator.find() as b:
            touch_sensor = TouchSensor(b, nxt.sensor.Port.S1)

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(touch_sensor)

            try:
                executor.spin()
            finally:
                executor.shutdown()

                for node in executor._nodes:
                    node.destroy()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
