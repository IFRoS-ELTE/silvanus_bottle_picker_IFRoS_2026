#!/usr/bin/env python3
"""Drive the base to a standoff pose in front of the closest detected bottle.

Subscribes to /bottles/positions (PoseArray, usually in base_footprint), transforms
to map, picks the nearest bottle to the robot, and sends NavigateToPose to a point
``standoff_distance`` metres short of the bottle, yaw facing it.

Guards (from real-robot bag failures):
  - ignore bottles farther than ``max_bottle_distance`` (base_footprint)
  - require ``stable_count`` consistent nearest bottles before committing
  - reject standoff goals outside the /map grid
  - lock the first committed goal (no retarget spam on abort / flicker)

With ``use_arm_poses:=true`` (default): MoveIt takes the arm to the **survey** pose
before listening for bottles, then to the **near** pose after Nav2 succeeds.

  ros2 run grasp_pipeline nav_to_bottle --ros-args \
    --params-file .../arm_poses_robot.yaml \
    -p standoff_distance:=0.55
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from grasp_pipeline.arm_pose import (
    ARM_JOINTS,
    DEFAULT_NEAR_DEG,
    DEFAULT_SURVEY_DEG,
    deg_to_rad,
)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class NavToBottle(Node):
    def __init__(self) -> None:
        super().__init__('nav_to_bottle')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('standoff_distance', 0.55)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('bottles_topic', '/bottles/positions')
        self.declare_parameter('goal_action', 'navigate_to_pose')
        self.declare_parameter('already_there_extra', 0.10)
        self.declare_parameter('oneshot', True)
        self.declare_parameter('use_arm_poses', True)
        self.declare_parameter('survey_joints_deg', DEFAULT_SURVEY_DEG)
        self.declare_parameter('near_joints_deg', DEFAULT_NEAR_DEG)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('arm_group', 'xarm6')
        # Bag fixes: ghost bottles at 4 m + goals past map +x.
        self.declare_parameter('max_bottle_distance', 2.5)
        self.declare_parameter('stable_count', 3)
        self.declare_parameter('stable_xy_tol', 0.25)
        self.declare_parameter('map_margin_m', 0.15)
        self.declare_parameter('max_nav_retries', 2)

        self.standoff = float(self.get_parameter('standoff_distance').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.already_extra = float(self.get_parameter('already_there_extra').value)
        self.oneshot = bool(self.get_parameter('oneshot').value)
        self.use_arm_poses = bool(self.get_parameter('use_arm_poses').value)
        self.max_bottle_dist = float(self.get_parameter('max_bottle_distance').value)
        self.stable_need = max(1, int(self.get_parameter('stable_count').value))
        self.stable_tol = float(self.get_parameter('stable_xy_tol').value)
        self.map_margin = float(self.get_parameter('map_margin_m').value)
        self.max_nav_retries = max(0, int(self.get_parameter('max_nav_retries').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('goal_action').value),
            callback_group=self.cb_group)
        self.move_group = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)

        self._busy = False
        self._done = False
        self._accept_bottles = not self.use_arm_poses
        self._booted = False
        self._arm_busy = False
        self._arm_wait_timer = None
        self._latest: Optional[PoseArray] = None
        self._last_bottles_wall = 0.0
        self._stable_xy: Optional[Tuple[float, float]] = None
        self._stable_hits = 0
        self._locked_goal: Optional[PoseStamped] = None
        self._nav_retries = 0
        self._map_info = None  # OccupancyGrid.info

        topic = str(self.get_parameter('bottles_topic').value)
        self._bottles_topic = topic
        self.create_subscription(
            PoseArray, topic, self.on_bottles, 10, callback_group=self.cb_group)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 1, callback_group=self.cb_group)
        self.create_timer(5.0, self._watchdog, callback_group=self.cb_group)
        self.create_timer(0.5, self._tick, callback_group=self.cb_group)
        self._boot_timer = self.create_timer(
            0.2, self._boot, callback_group=self.cb_group)
        self.get_logger().info(
            f'Waiting for bottles on {topic}; standoff={self.standoff:.2f} m '
            f'in frame {self.map_frame}; max_range={self.max_bottle_dist:.1f} m; '
            f'stable_count={self.stable_need}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_info = msg.info
        self.get_logger().info(
            f'Map bounds ready: {msg.info.width}x{msg.info.height} @ '
            f'{msg.info.resolution:.3f} m, origin='
            f'({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f})',
            once=True)

    def _in_map(self, x: float, y: float) -> bool:
        info = self._map_info
        if info is None:
            return True  # allow until /map arrives; TF still required
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        m = self.map_margin
        xmin, ymin = ox + m, oy + m
        xmax = ox + info.width * res - m
        ymax = oy + info.height * res - m
        return xmin <= x <= xmax and ymin <= y <= ymax

    def _boot(self) -> None:
        if self._booted:
            return
        self._booted = True
        self._boot_timer.cancel()
        if not self.use_arm_poses:
            self._accept_bottles = True
            self.get_logger().info('Arm poses disabled — looking for bottles')
            return
        self._start_arm(
            self.get_parameter('survey_joints_deg').value,
            'survey pose',
            self._on_survey_done)

    def _on_survey_done(self, ok: bool, err: str = '') -> None:
        if ok:
            self._accept_bottles = True
            self.get_logger().info('Survey pose ready — looking for bottles')
        else:
            self.get_logger().error(f'Survey failed: {err}')
            self.get_logger().warn(
                'Continuing without survey pose — still looking for bottles')
            self._accept_bottles = True

    def _watchdog(self) -> None:
        if self._done or self._busy:
            return
        age = time.monotonic() - self._last_bottles_wall if self._last_bottles_wall else None
        if not self._accept_bottles:
            self.get_logger().warn(
                'Still moving arm to survey (or survey failed) — not accepting bottles yet')
            return
        if age is None:
            self.get_logger().warn(
                f'No messages yet on {self._bottles_topic}. '
                'Is bottle_localizer running? Check: ros2 topic hz /bottles/detections '
                'and ros2 topic hz /bottles/positions')
        elif age > 5.0:
            self.get_logger().warn(
                f'Last {self._bottles_topic} was {age:.0f}s ago (empty or TF failing?). '
                'Confirm map→base_footprint (2D Pose Estimate) and localizer TF.')

    def point_in_map(self, frame: str, x: float, y: float, z: float = 0.0
                     ) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, frame, Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as exc:
            self.get_logger().warn(f'TF {frame} -> {self.map_frame}: {exc}',
                                   throttle_duration_sec=2.0)
            return None
        pt = PointStamped()
        pt.header.frame_id = frame
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        out = do_transform_point(pt, tf)
        return (out.point.x, out.point.y)

    def robot_xy(self) -> Optional[Tuple[float, float]]:
        return self.point_in_map(self.robot_frame, 0.0, 0.0)

    def _make_arm_goal(self, joints_deg) -> MoveGroup.Goal:
        positions = deg_to_rad(joints_deg)
        c = Constraints()
        for joint_name, value in zip(ARM_JOINTS, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=joint_name, position=float(value),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = str(self.get_parameter('arm_group').value)
        req.num_planning_attempts = int(self.get_parameter('planning_attempts').value)
        req.allowed_planning_time = float(self.get_parameter('planning_time').value)
        vel = float(self.get_parameter('velocity_scaling').value)
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = vel
        req.start_state.is_diff = True
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        return goal

    def _start_arm(self, joints_deg, what: str,
                   done_cb: Callable[[bool, str], None]) -> None:
        if self._arm_busy:
            done_cb(False, 'arm already busy')
            return
        if not self.move_group.server_is_ready():
            if not self.move_group.wait_for_server(timeout_sec=0.0):
                self.get_logger().info(f'Waiting for /move_action ({what})...')
                self._arm_wait_deadline = time.monotonic() + 10.0
                self._arm_wait_what = what
                self._arm_wait_joints = joints_deg
                self._arm_wait_cb = done_cb
                if self._arm_wait_timer is None:
                    self._arm_wait_timer = self.create_timer(
                        0.5, self._poll_move_server, callback_group=self.cb_group)
                return
        self._send_arm_goal(joints_deg, what, done_cb)

    def _poll_move_server(self) -> None:
        if self.move_group.server_is_ready():
            self._arm_wait_timer.cancel()
            self._arm_wait_timer = None
            self._send_arm_goal(
                self._arm_wait_joints, self._arm_wait_what, self._arm_wait_cb)
            return
        if time.monotonic() > self._arm_wait_deadline:
            self._arm_wait_timer.cancel()
            self._arm_wait_timer = None
            self._arm_wait_cb(
                False, '/move_action not available (is silvanus-up --arm running?)')

    def _send_arm_goal(self, joints_deg, what: str,
                       done_cb: Callable[[bool, str], None]) -> None:
        self._arm_busy = True
        self.get_logger().info(f'Arm: {what}')
        goal = self._make_arm_goal(joints_deg)
        send = self.move_group.send_goal_async(goal)

        def _on_goal(fut) -> None:
            handle = fut.result()
            if handle is None or not handle.accepted:
                self._arm_busy = False
                done_cb(False, f'{what}: MoveIt goal rejected')
                return
            result_fut = handle.get_result_async()

            def _on_result(rfut) -> None:
                self._arm_busy = False
                try:
                    result = rfut.result().result
                except Exception as exc:  # noqa: BLE001
                    done_cb(False, f'{what}: {exc}')
                    return
                if result.error_code.val != MoveItErrorCodes.SUCCESS:
                    done_cb(False, f'{what}: MoveIt error {result.error_code.val}')
                    return
                self.get_logger().info(f'Arm done: {what}')
                done_cb(True, '')

            result_fut.add_done_callback(_on_result)

        send.add_done_callback(_on_goal)

    def _finish_ok(self, msg: str) -> None:
        self.get_logger().info(msg)
        self._done = True
        if self.oneshot:
            self.create_timer(0.1, self._shutdown)

    def _shutdown(self) -> None:
        raise SystemExit(0)

    def on_bottles(self, msg: PoseArray) -> None:
        self._last_bottles_wall = time.monotonic()
        self._latest = msg
        self.get_logger().info(
            f'Got {len(msg.poses)} bottle pose(s) on {self._bottles_topic} '
            f'(frame={msg.header.frame_id or "?"})',
            throttle_duration_sec=2.0)

    def _pick_best_bottle(self, msg: PoseArray, robot: Tuple[float, float]
                          ) -> Optional[Tuple[float, float, float]]:
        """Return (bx, by, dist) of nearest in-range bottle in map, or None."""
        frame = msg.header.frame_id or self.robot_frame
        best = None
        for pose in msg.poses:
            # Range gate in the bottle's native frame (usually base_footprint).
            if frame in ('', self.robot_frame, 'base_link'):
                r = math.hypot(pose.position.x, pose.position.y)
                if r > self.max_bottle_dist or r < 0.15:
                    continue
            xy = self.point_in_map(frame, pose.position.x, pose.position.y)
            if xy is None:
                continue
            bx, by = xy
            if not self._in_map(bx, by):
                self.get_logger().warn(
                    f'Skip bottle map=({bx:.2f},{by:.2f}) — outside /map',
                    throttle_duration_sec=2.0)
                continue
            d2 = (bx - robot[0]) ** 2 + (by - robot[1]) ** 2
            if best is None or d2 < best[0]:
                best = (d2, bx, by)
        if best is None:
            return None
        d2, bx, by = best
        return bx, by, math.sqrt(d2)

    def _update_stable(self, bx: float, by: float) -> bool:
        if self._stable_xy is None:
            self._stable_xy = (bx, by)
            self._stable_hits = 1
        else:
            sx, sy = self._stable_xy
            if math.hypot(bx - sx, by - sy) <= self.stable_tol:
                self._stable_hits += 1
                # slow track
                a = 0.3
                self._stable_xy = ((1 - a) * sx + a * bx, (1 - a) * sy + a * by)
            else:
                self.get_logger().info(
                    f'Bottle jumped ({sx:.2f},{sy:.2f})→({bx:.2f},{by:.2f}); '
                    'resetting stability',
                    throttle_duration_sec=2.0)
                self._stable_xy = (bx, by)
                self._stable_hits = 1
        return self._stable_hits >= self.stable_need

    def _tick(self) -> None:
        if self._busy or self._done or not self._accept_bottles or self._arm_busy:
            return
        # If we already locked a goal and are retrying after abort, resend it.
        if self._locked_goal is not None and self._nav_retries > 0:
            self._busy = True
            self.get_logger().info(
                f'Retrying locked NavigateToPose ({self._nav_retries} left)...')
            self._send_nav(self._locked_goal)
            return

        msg = self._latest
        if msg is None:
            return
        if not msg.poses:
            self.get_logger().warn(
                'Got /bottles/positions with 0 poses (localizer sees no bottles)',
                throttle_duration_sec=5.0)
            return

        robot = self.robot_xy()
        if robot is None:
            self.get_logger().warn(
                'Have /bottles/positions but no map→base_footprint. '
                'In RViz set 2D Pose Estimate first (AMCL must be localized).',
                throttle_duration_sec=3.0)
            return

        picked = self._pick_best_bottle(msg, robot)
        if picked is None:
            self.get_logger().warn(
                f'No in-range bottle (max {self.max_bottle_dist:.1f} m) inside map.',
                throttle_duration_sec=3.0)
            return
        bx, by, dist = picked

        if not self._update_stable(bx, by):
            self.get_logger().info(
                f'Stabilizing bottle map=({bx:.2f},{by:.2f}) '
                f'{self._stable_hits}/{self.stable_need}',
                throttle_duration_sec=1.0)
            return
        bx, by = self._stable_xy  # type: ignore[misc]

        if dist <= self.standoff + self.already_extra:
            self._busy = True
            if self.use_arm_poses:
                self._start_arm(
                    self.get_parameter('near_joints_deg').value,
                    'near pose',
                    lambda ok, err: self._after_near_already_there(ok, err, dist))
            else:
                self._finish_ok(
                    f'Already within standoff ({dist:.2f} m). '
                    'Start pick_bottle when ready.')
            return

        rx, ry = robot
        dx, dy = bx - rx, by - ry
        ux, uy = dx / dist, dy / dist
        gx = bx - self.standoff * ux
        gy = by - self.standoff * uy
        yaw = math.atan2(by - gy, bx - gx)

        if not self._in_map(gx, gy):
            self.get_logger().error(
                f'Standoff goal ({gx:.2f},{gy:.2f}) outside /map — not sending. '
                'Move the robot / bottle inward or expand the map.')
            self._stable_xy = None
            self._stable_hits = 0
            return

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = Time().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quat(yaw)

        self.get_logger().info(
            f'Closest bottle map=({bx:.2f},{by:.2f}) dist={dist:.2f} m → '
            f'standoff goal=({gx:.2f},{gy:.2f}) yaw={yaw:.2f} rad (LOCKED)')
        self._locked_goal = goal
        self._nav_retries = self.max_nav_retries
        self._busy = True
        self._send_nav(goal)

    def _after_near_already_there(self, ok: bool, err: str, dist: float) -> None:
        if not ok:
            self.get_logger().error(f'near pose failed: {err}')
            self._busy = False
            return
        self._finish_ok(
            f'Already within standoff ({dist:.2f} m); near pose set. '
            'Start pick_bottle when ready.')

    def _send_nav(self, pose: PoseStamped) -> None:
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'navigate_to_pose not ready yet — will retry',
                throttle_duration_sec=3.0)
            self._busy = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info('Sending NavigateToPose ...')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('NavigateToPose rejected')
            self._on_nav_fail()
            return
        self.get_logger().info('NavigateToPose accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_nav_fail(self) -> None:
        if self._nav_retries > 0 and self._locked_goal is not None:
            self._nav_retries -= 1
            self._busy = False  # _tick will resend locked goal
            self.get_logger().warn(
                f'Nav failed — will retry locked goal ({self._nav_retries} left)')
            return
        self.get_logger().error(
            'NavigateToPose failed and retries exhausted — stopping (no retarget). '
            'Re-run nav_to_bottle when the bottle is stable and inside the map.')
        self._done = True
        self._busy = False

    def _on_result(self, future) -> None:
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            if self.use_arm_poses:
                self._start_arm(
                    self.get_parameter('near_joints_deg').value,
                    'near pose',
                    self._after_near_nav)
            else:
                self._finish_ok(
                    'Goal succeeded — base at standoff. '
                    'Start pick_bottle when ready.')
        else:
            self.get_logger().error(f'NavigateToPose finished with status={status}')
            self._on_nav_fail()

    def _after_near_nav(self, ok: bool, err: str) -> None:
        if not ok:
            self.get_logger().error(f'near pose failed: {err}')
            self._busy = False
            return
        self._finish_ok(
            'Goal succeeded — base at standoff, arm at near pose. '
            'Start pick_bottle when ready.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavToBottle()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
