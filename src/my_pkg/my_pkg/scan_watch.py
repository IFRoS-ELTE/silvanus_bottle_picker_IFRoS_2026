"""Read the lidar and print the nearest obstacle.

Start here. This node touches no actuator, so it is safe to run at any time,
and it proves the whole chain works: your container discovered the robot, DDS
is delivering messages, and your QoS settings are right.

    ros2 run my_pkg scan_watch
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanWatch(Node):
    def __init__(self) -> None:
        super().__init__('scan_watch')

        # qos_profile_sensor_data, NOT the default. Sensor drivers publish
        # BEST_EFFORT; a default (RELIABLE) subscription silently matches
        # nothing. This is the number one reason a topic shows up in
        # `ros2 topic list` and then never delivers a single message.
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info('waiting for /scan ...')

    def on_scan(self, msg: LaserScan) -> None:
        nearest_range = None
        nearest_angle = 0.0

        for index, measured in enumerate(msg.ranges):
            # inf means "nothing out there", nan means "bad reading". Both are
            # normal in a real scan and both poison min() if you forget them.
            if not math.isfinite(measured):
                continue
            if measured < msg.range_min or measured > msg.range_max:
                continue
            if nearest_range is None or measured < nearest_range:
                nearest_range = measured
                nearest_angle = msg.angle_min + index * msg.angle_increment

        if nearest_range is None:
            self.get_logger().warn(
                'scan has no usable returns — an empty room can legitimately '
                'look like this', throttle_duration_sec=5.0)
            return

        self.get_logger().info(
            f'nearest obstacle {nearest_range:.2f} m '
            f'at {math.degrees(nearest_angle):+.0f} deg '
            f'({len(msg.ranges)} beams, frame {msg.header.frame_id})',
            throttle_duration_sec=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
