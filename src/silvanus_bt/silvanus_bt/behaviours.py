"""Custom behaviours (leaves) of the Silvanus tree.

py_trees lifecycle reminder:
  __init__   -> the tree is built (no ROS yet). Register the blackboard keys the behaviour
                reads or writes here.
  setup      -> once, at start-up: create subscriptions/publishers with the node here.
  initialise -> every time the behaviour STARTS (goes from not-RUNNING to being ticked).
  update     -> on every tick while active: returns SUCCESS / FAILURE / RUNNING.
  terminate  -> when it finishes or the tree interrupts it (new_status INVALID).

Blackboard keys:
  /tracks     list[Track]           tracked bottles, in `map`              (BottlesToBB)
  /robot_xy   (x, y) | None         robot position in `map`                (BottlesToBB)
  /map_info   MapMetaData | None    map bounds                             (BottlesToBB)
  /blacklist  list[BlacklistEntry]  places not to retry                    (MarkTarget)
  /target     Target | None         chosen bottle                          (SelectTarget)
  /nav_goal   NavigateToPose.Goal   read by the Nav2 action client         (ComputeApproachGoal)
  /arm_goal   MoveGroup.Goal        read by the MoveIt action client       (SetArmGoal)
"""

from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Access, Status

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from silvanus_bt.config import Config
from silvanus_bt.tracking import (BlacklistEntry, BottleTracker, Target, choose_target,
                                  inside_map_bounds, prune_blacklist, standoff_pose)

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def _now(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------------------------
# Data -> blackboard
# ---------------------------------------------------------------------------------------------
class BottlesToBB(py_trees.behaviour.Behaviour):
    """Reads the localizer's bottles, moves them to `map`, tracks them, writes the blackboard.

    Always returns SUCCESS: it is a "sensor", it decides nothing.
    """

    def __init__(self, cfg: Config, name: str = 'Bottles2BB') -> None:
        super().__init__(name)
        self.cfg = cfg
        self.bb = self.attach_blackboard_client(name=name)
        for key in ('tracks', 'robot_xy', 'map_info', 'blacklist'):
            self.bb.register_key(key=key, access=Access.WRITE)
        self.tracker = BottleTracker(cfg.assoc_radius, forget_after=cfg.forget_after)
        self._pending = None

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.node.create_subscription(Detection3DArray, self.cfg.detections_topic,
                                      self._on_detections, 10)
        # /map from slam_toolbox / map_server is latched (transient local)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.bb.tracks = []
        self.bb.robot_xy = None
        self.bb.map_info = None
        self.bb.blacklist = []

    def _on_detections(self, msg: Detection3DArray) -> None:
        self._pending = msg       # processed on the next tick (update)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.bb.map_info = msg.info

    def _lookup(self, frame: str):
        try:
            return self.tf_buffer.lookup_transform(self.cfg.map_frame, frame, Time())
        except TransformException as exc:
            self.node.get_logger().warn(f'TF {frame} -> {self.cfg.map_frame}: {exc}',
                                        throttle_duration_sec=3.0)
            return None

    def update(self) -> Status:
        now = _now(self.node)
        tf_robot = self._lookup(self.cfg.robot_frame)
        if tf_robot is None:
            self.bb.robot_xy = None
            self.feedback_message = 'no map->robot TF (SLAM / localization running?)'
        else:
            t = tf_robot.transform.translation
            self.bb.robot_xy = (t.x, t.y)

        msg, self._pending = self._pending, None
        dets = []
        if msg is not None and msg.detections:
            tf = self._lookup(msg.header.frame_id or self.cfg.robot_frame)
            if tf is not None:
                for det in msg.detections:
                    score = det.results[0].hypothesis.score if det.results else 1.0
                    if score < self.cfg.min_confidence:
                        continue
                    p = PointStamped()
                    p.point = det.bbox.center.position
                    out = do_transform_point(p, tf)
                    dets.append((out.point.x, out.point.y, float(score)))
        if msg is not None:
            self.tracker.update(dets, now)

        self.bb.tracks = list(self.tracker.tracks)
        self.bb.blacklist = prune_blacklist(self.bb.blacklist, now)
        if self.bb.robot_xy is not None:
            self.feedback_message = f'{len(self.tracker.tracks)} bottle(s) tracked'
        return Status.SUCCESS


# ---------------------------------------------------------------------------------------------
# Conditions
# ---------------------------------------------------------------------------------------------
class SelectTarget(py_trees.behaviour.Behaviour):
    """CONDITION: is there a stable bottle to collect? If so, write it to /target."""

    def __init__(self, cfg: Config, name: str = 'BottleToCollect?') -> None:
        super().__init__(name)
        self.cfg = cfg
        self.bb = self.attach_blackboard_client(name=name)
        for key in ('tracks', 'robot_xy', 'blacklist'):
            self.bb.register_key(key=key, access=Access.READ)
        self.bb.register_key(key='target', access=Access.WRITE)

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.bb.target = None

    def update(self) -> Status:
        if self.bb.robot_xy is None:
            self.feedback_message = 'no robot pose (map->base_footprint TF)'
            self.node.get_logger().warn(
                'Cannot choose a bottle: map->base_footprint TF missing. '
                'Check: ros2 run tf2_ros tf2_echo map base_footprint',
                throttle_duration_sec=5.0)
            return Status.FAILURE
        best = choose_target(
            self.bb.tracks, self.bb.robot_xy, self.bb.blacklist, _now(self.node),
            min_hits=self.cfg.min_hits, max_range=self.cfg.max_bottle_distance,
            blacklist_radius=self.cfg.blacklist_radius, selection=self.cfg.selection)
        if best is None:
            self.feedback_message = 'no stable bottle'
            return Status.FAILURE
        self.bb.target = Target(best.id, best.x, best.y, best.conf)
        self.feedback_message = (f'bottle #{best.id} ({best.x:.2f},{best.y:.2f}) '
                                 f'conf {best.conf:.2f}')
        self.node.get_logger().info(f'Target chosen: {self.feedback_message} '
                                    f'[selection: {self.cfg.selection}]')
        return Status.SUCCESS


class TargetWithin(py_trees.behaviour.Behaviour):
    """CONDITION: is the robot already within `distance` m of the target bottle?"""

    def __init__(self, cfg: Config, distance: float, name: str) -> None:
        super().__init__(name)
        self.cfg = cfg
        self.distance = distance
        self.bb = self.attach_blackboard_client(name=name)
        for key in ('target', 'robot_xy'):
            self.bb.register_key(key=key, access=Access.READ)

    def update(self) -> Status:
        t, r = self.bb.target, self.bb.robot_xy
        if t is None or r is None:
            return Status.FAILURE
        d = _dist((t.x, t.y), r)
        self.feedback_message = f'{d:.2f} m away (threshold {self.distance:.2f})'
        return Status.SUCCESS if d <= self.distance else Status.FAILURE


# ---------------------------------------------------------------------------------------------
# Instant actions
# ---------------------------------------------------------------------------------------------
class SetExplore(py_trees.behaviour.Behaviour):
    """Pauses (False) or resumes (True) explore_lite by publishing on explore/resume."""

    def __init__(self, cfg: Config, resume: bool, name: str) -> None:
        super().__init__(name)
        self.cfg = cfg
        self.resume = resume

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.pub = self.node.create_publisher(Bool, self.cfg.explore_resume_topic, 10)

    def update(self) -> Status:
        if not self.cfg.use_explore:
            self.feedback_message = 'exploration disabled (use_explore:=false)'
            return Status.SUCCESS
        self.pub.publish(Bool(data=self.resume))
        self.node.get_logger().info('Exploration ' + ('RESUMED' if self.resume else 'PAUSED'))
        return Status.SUCCESS


class ComputeApproachGoal(py_trees.behaviour.Behaviour):
    """Computes the pose `distance` m from the bottle, facing it, and writes /nav_goal.

    First it refreshes the bottle position from its track (if still seen): in the second
    approach step this uses the measurement taken up close, which is more reliable.
    """

    def __init__(self, cfg: Config, distance: float, name: str) -> None:
        super().__init__(name)
        self.cfg = cfg
        self.distance = distance
        self.bb = self.attach_blackboard_client(name=name)
        for key in ('tracks', 'robot_xy', 'map_info'):
            self.bb.register_key(key=key, access=Access.READ)
        self.bb.register_key(key='target', access=Access.WRITE)
        self.bb.register_key(key='nav_goal', access=Access.WRITE)

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']

    def update(self) -> Status:
        target, robot = self.bb.target, self.bb.robot_xy
        if target is None or robot is None:
            self.feedback_message = 'no target or no robot pose'
            return Status.FAILURE
        for tr in self.bb.tracks:
            if tr.id == target.track_id:
                target.x, target.y, target.conf = tr.x, tr.y, tr.conf
                self.bb.target = target
                break

        gx, gy, yaw = standoff_pose(robot, (target.x, target.y), self.distance)
        if self.cfg.check_map_bounds and not inside_map_bounds(
                gx, gy, self.bb.map_info, self.cfg.map_margin):
            self.feedback_message = f'goal ({gx:.2f},{gy:.2f}) outside /map'
            self.node.get_logger().warn(self.feedback_message)
            return Status.FAILURE

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.cfg.map_frame
        # Stamp 0 = "latest TF" in Nav2 (laptop and robot clocks are not synchronised,
        # so the laptop's time is not used)
        goal.pose.header.stamp = Time().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        self.bb.nav_goal = goal
        self.feedback_message = (f'go to ({gx:.2f},{gy:.2f}) yaw {math.degrees(yaw):.0f} deg, '
                                 f'{self.distance:.2f} m from the bottle')
        self.node.get_logger().info(f'{self.name}: {self.feedback_message}')
        return Status.SUCCESS


class SetArmGoal(py_trees.behaviour.Behaviour):
    """Writes the MoveIt goal for the 'survey' or 'near' pose to /arm_goal."""

    def __init__(self, cfg: Config, pose: str, name: str) -> None:
        super().__init__(name)
        self.cfg = cfg
        self.pose = pose
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key='arm_goal', access=Access.WRITE)

    def update(self) -> Status:
        joints_deg = (self.cfg.survey_joints_deg if self.pose == 'survey'
                      else self.cfg.near_joints_deg)
        c = Constraints()
        for joint, deg in zip(ARM_JOINTS, joints_deg):
            c.joint_constraints.append(JointConstraint(
                joint_name=joint, position=math.radians(deg),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg.arm_group
        req.num_planning_attempts = self.cfg.planning_attempts
        req.allowed_planning_time = self.cfg.planning_time
        req.max_velocity_scaling_factor = self.cfg.velocity_scaling
        req.max_acceleration_scaling_factor = self.cfg.velocity_scaling
        req.start_state.is_diff = True
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        self.bb.arm_goal = goal
        return Status.SUCCESS


class MarkTarget(py_trees.behaviour.Behaviour):
    """Blacklists the target bottle (forever if collected, `retry_after` s if it failed)
    and clears /target."""

    def __init__(self, cfg: Config, permanent: bool, name: str) -> None:
        super().__init__(name)
        self.cfg = cfg
        self.permanent = permanent
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key='target', access=Access.WRITE)
        self.bb.register_key(key='blacklist', access=Access.WRITE)

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']

    def update(self) -> Status:
        t = self.bb.target
        if t is not None:
            until = None if self.permanent else _now(self.node) + self.cfg.retry_after
            reason = 'collected' if self.permanent else 'failed'
            self.bb.blacklist = self.bb.blacklist + [BlacklistEntry(t.x, t.y, until, reason)]
            self.node.get_logger().info(
                f'Bottle #{t.track_id} ({t.x:.2f},{t.y:.2f}) blacklisted ({reason})')
        self.bb.target = None
        return Status.SUCCESS


# ---------------------------------------------------------------------------------------------
# Long actions
# ---------------------------------------------------------------------------------------------
class WaitForPick(py_trees.behaviour.Behaviour):
    """pick_mode 'wait' (manual pick): waits for a std_msgs/Bool on /bt/pick_result
    (True = collected, False = failed). pick_mode 'skip': SUCCESS straight away.
    (pick_mode 'action' uses an ActionClient on /pick_bottle instead, see tree.py.)
    """

    def __init__(self, cfg: Config, name: str = 'WaitForPick') -> None:
        super().__init__(name)
        self.cfg = cfg
        self._result = None

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.node.create_subscription(Bool, self.cfg.pick_result_topic, self._on_result, 10)

    def _on_result(self, msg: Bool) -> None:
        self._result = bool(msg.data)

    def initialise(self) -> None:
        self._result = None
        self._start = time.monotonic()
        if self.cfg.pick_mode != 'skip':
            self.node.get_logger().info(
                'In position. Run pick-bottle and when it is done: '
                f'ros2 topic pub --once {self.cfg.pick_result_topic} std_msgs/msg/Bool '
                '"{data: true}"   (false if it failed)')

    def update(self) -> Status:
        if self.cfg.pick_mode == 'skip':
            return Status.SUCCESS
        if self._result is not None:
            return Status.SUCCESS if self._result else Status.FAILURE
        if self.cfg.pick_timeout > 0 and time.monotonic() - self._start > self.cfg.pick_timeout:
            self.feedback_message = 'timed out waiting for the pick'
            return Status.FAILURE
        self.feedback_message = f'waiting for {self.cfg.pick_result_topic}'
        return Status.RUNNING
