"""Drive the base forward briefly, then stop. Copy this shape for any node
that commands motion.

THIS MOVES THE REAL ROBOT. It refuses to run unless you explicitly enable it,
and an operator must be at the emergency stop before you do:

    ros2 run my_pkg safe_drive --ros-args -p enable:=true

Four things here are not optional, and you should carry all four into your
own motion code:

1. It publishes geometry_msgs/msg/Twist. The physical Scout driver subscribes
   to plain Twist. The simulator and stock Nav2 use TwistStamped — publish
   that here and the robot silently ignores you.
2. It stops itself after a fixed duration. A control loop with no time limit
   is a robot that keeps going when your attention lapses.
3. It watches /scan and stops if anything is close.
4. It publishes explicit zeros on every exit path. A publisher that stops
   publishing is NOT a stop command: the driver keeps executing the last
   velocity it was given.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CONTROL_PERIOD_S = 0.05  # 20 Hz


class SafeDrive(Node):
    def __init__(self) -> None:
        super().__init__('safe_drive')

        # Default false, so an accidental `ros2 run` moves nothing.
        self.declare_parameter('enable', False)
        self.declare_parameter('speed', 0.05)          # m/s
        self.declare_parameter('duration', 3.0)        # s
        self.declare_parameter('stop_distance', 0.8)   # m

        self.enable = self.get_parameter('enable').value
        self.speed = float(self.get_parameter('speed').value)
        self.duration = float(self.get_parameter('duration').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.finished = False
        self.blocked = False
        self.started_at = self.get_clock().now()

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        if not self.enable:
            self.get_logger().warn(
                'enable is false, so this node will command nothing. Re-run '
                'with -p enable:=true once an operator is at the e-stop.')
            self.finished = True
            return

        if self.speed > 0.2:
            self.get_logger().error(
                f'speed {self.speed} m/s is above the 0.2 m/s cap this '
                'template will command. Refusing to move.')
            self.finished = True
            return

        self.get_logger().warn(
            f'MOVING: {self.speed} m/s for {self.duration} s. E-stop ready?')
        self.create_timer(CONTROL_PERIOD_S, self.on_tick)

    def on_scan(self, msg: LaserScan) -> None:
        close = [
            r for r in msg.ranges
            if msg.range_min <= r <= min(msg.range_max, self.stop_distance)
        ]
        # inf/nan fail every comparison above, so they drop out on their own.
        was_blocked, self.blocked = self.blocked, bool(close)
        if self.blocked and not was_blocked:
            self.get_logger().warn(
                f'obstacle within {self.stop_distance} m — holding still')

    def on_tick(self) -> None:
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.get_logger().info('duration reached')
            self.finished = True
            return

        command = Twist()
        if not self.blocked:
            command.linear.x = self.speed
        self.cmd.publish(command)

    def stop(self) -> None:
        """Zero the base. Called on the normal path AND on Ctrl-C."""
        for _ in range(10):
            self.cmd.publish(Twist())
        self.get_logger().info('sent zero velocity')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeDrive()
    try:
        # Not rclpy.spin(): this loop exits when the node says it is done,
        # so stop() below always runs before the process ends.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
