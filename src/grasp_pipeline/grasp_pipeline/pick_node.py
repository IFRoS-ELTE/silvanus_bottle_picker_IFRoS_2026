"""Grasp a bottle with the xArm6 using the grasps from grasp_inference, and lift it.

    ros2 run grasp_pipeline pick_bottle --ros-args -p use_sim_time:=true

Needs move_group (ros2 launch grasp_pipeline moveit_sim.launch.py) and the sim's
xarm6 / xarm_gripper trajectory controllers.

Sequence:
  1. Open the gripper and move the arm to the pick view pose (camera looking at the ground
     just in front of the robot, the only area the arm can reach).
  2. Locate the bottle from estimates made after the arm settled: its position is the median
     of bottle_localizer's /bottles/positions (YOLO box + median depth, accurate even with a
     noisy depth camera); standing or lying, its direction and length come from
     grasp_inference's /grasp/bottle cylinder fit. Its radius is taken as `bottle_radius`.
  3. Body grasp, perpendicular to the bottle: horizontal approaches at `body_grasp_height` of
     a standing bottle (fanning out from the robot's side, then tilted down), or approaches
     from above onto the middle of a lying one, all closing across the bottle. Poses are
     filtered with inverse kinematics, then MoveIt plans to a pre-grasp in front of the first
     reachable one, with the bottle in its world. Only with use_graspnet:=true, if none is
     reachable: Contact-GraspNet's candidates, ranked by distance to the bottle axis.
  4. Move straight in along the approach axis to the grasp point.
  5. Close the gripper, squeezing (see move_gripper); if the fingers close all the way,
     there is nothing between them: open, back straight out and fail the attempt.
  6. Lift `lift_height` straight up (or back along the approach if the arm can't go
     straight up from there), then check the fingers didn't creep shut (bottle slipped out).
If a step fails it returns to the view pose and waits for new grasps, up to
`max_attempts` times.

Contact-GraspNet grasp frame: z = approach, x = finger closing direction, and the point
between the fingers is `cgn_gripper_depth` along z from the origin.
xArm gripper link_tcp: z = approach, fingers close along y.
"""

from collections import namedtuple
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
import tf2_ros

from grasp_pipeline.geometry import matrix_to_pose, pose_to_matrix, transform_to_matrix

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINT = 'drive_joint'

# Contact-GraspNet closes along x, the xArm gripper along y: rotate -90 deg about z
CGN_TO_TCP = np.array([[0.0, 1.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])
# A parallel gripper is symmetric under a half turn about the approach axis
HALF_TURN_Z = np.diag([-1.0, -1.0, 1.0])


# A bottle estimate in the planning frame
Bottle = namedtuple('Bottle', 'centre axis radius length standing')


class PickFailed(Exception):
    pass


class PathInfeasible(PickFailed):
    pass


class PickBottle(Node):
    def __init__(self):
        super().__init__('pick_bottle')

        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('group', 'xarm6')
        self.declare_parameter('tcp_link', 'link_tcp')
        # Camera on the robot's right, looking 45 deg down at x ~ 0.6 m in front of
        # base_footprint. Straight down, YOLO doesn't recognise the bottle; from behind, the
        # front bumper hides it.
        self.declare_parameter('view_joints', [-0.8940, 1.4845, -2.4225, 0.4590, 1.5971, -2.6120])
        self.declare_parameter('settle_time', 2.0)           # s to wait at the view pose
        self.declare_parameter('cgn_gripper_depth', 0.1034)  # Contact-GraspNet origin -> fingers
        self.declare_parameter('grasp_depth_offset', 0.01)   # extra push along the approach (m)
        self.declare_parameter('pre_grasp_distance', 0.10)
        self.declare_parameter('max_approach_up', 0.3)       # reject grasps approaching upwards
        # Finger midpoint further than this from the bottle axis: candidate rejected (m).
        # Generous because the sim depth camera is very noisy (see README)
        self.declare_parameter('max_axis_distance', 0.05)
        self.declare_parameter('max_candidates', 3)          # candidates tried for the pre-grasp
        # Contact-GraspNet grasps as a fallback when no body grasp is reachable
        self.declare_parameter('use_graspnet', False)
        # Bottle size: the radius isn't measured (too noisy in the sim); the length is, from
        # /grasp/bottle, with this as the default when there is no estimate
        self.declare_parameter('bottle_radius', 0.035)
        self.declare_parameter('bottle_length', 0.25)
        # Body grasp
        # Fraction of a standing bottle's height: high on the body, so less of the bottle stands
        # above the tilted gripper's palm (lower down, the palm hits it)
        self.declare_parameter('body_grasp_height', 0.6)
        self.declare_parameter('body_grasp_depth', 0.01)      # fingertips past the axis (m)
        # Approach directions around a standing bottle (deg from "straight away from the
        # robot") and downward tilts tried for each; tilts around a lying bottle's axis
        self.declare_parameter('body_yaws',
                               [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0])
        self.declare_parameter('body_tilts', [0.0, 10.0, 20.0, 30.0, 40.0])
        self.declare_parameter('lying_tilts', [0.0, 25.0, -25.0])
        self.declare_parameter('bottle_samples', 5)          # bottle positions combined
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_close', 0.85)
        # Fingers stopping this close to fully closed = nothing (or just a fingertip) in between
        self.declare_parameter('gripper_empty_margin', 0.1)
        # Squeezing fingers creeping more than this after the lift = the bottle slipped out
        self.declare_parameter('slip_tolerance', 0.004)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('max_attempts', 3)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame = p('planning_frame')
        self.group = p('group')
        self.tcp_link = p('tcp_link')
        self.view_joints = list(p('view_joints'))
        self.settle_time = float(p('settle_time'))
        self.cgn_depth = float(p('cgn_gripper_depth'))
        self.depth_offset = float(p('grasp_depth_offset'))
        self.pre_dist = float(p('pre_grasp_distance'))
        self.max_up = float(p('max_approach_up'))
        self.max_axis_dist = float(p('max_axis_distance'))
        self.max_candidates = int(p('max_candidates'))
        self.use_graspnet = bool(p('use_graspnet'))
        self.bottle_radius = float(p('bottle_radius'))
        self.bottle_length = float(p('bottle_length'))
        self.body_height = float(p('body_grasp_height'))
        self.body_depth = float(p('body_grasp_depth'))
        self.body_yaws = [math.radians(v) for v in p('body_yaws')]
        self.body_tilts = [math.radians(v) for v in p('body_tilts')]
        self.lying_tilts = [math.radians(v) for v in p('lying_tilts')]
        self.bottle_samples = int(p('bottle_samples'))
        self.grip_open = float(p('gripper_open'))
        self.grip_close = float(p('gripper_close'))
        self.empty_margin = float(p('gripper_empty_margin'))
        self.slip_tolerance = float(p('slip_tolerance'))
        self.lift = float(p('lift_height'))
        self.vel = float(p('velocity_scaling'))
        self.max_attempts = int(p('max_attempts'))

        cb = ReentrantCallbackGroup()
        self.move_group = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory',
                                    callback_group=cb)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/xarm6_traj_controller/follow_joint_trajectory',
                                callback_group=cb)
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    '/xarm_gripper_traj_controller/follow_joint_trajectory',
                                    callback_group=cb)
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path',
                                            callback_group=cb)
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                              callback_group=cb)
        self.ik = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=cb)
        self.create_subscription(PoseArray, '/grasp/candidates', self.on_grasp, 10,
                                 callback_group=cb)
        self.create_subscription(Marker, '/grasp/bottle', self.on_bottle, 10,
                                 callback_group=cb)
        self.create_subscription(PoseArray, '/bottles/positions', self.on_position, 10,
                                 callback_group=cb)
        # Estimates from images / clouds captured after accept_after
        self.bottles = []
        self.positions = []
        self.target_pub = self.create_publisher(PoseStamped, '/grasp/tcp_target', 10)

        self.grasp_lock = threading.Lock()
        self.accept_after = None  # only grasps from clouds newer than this are used
        self.grasp = None

        threading.Thread(target=self.run, daemon=True).start()

    # ---------------------------------------------------------------- inputs
    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def on_position(self, msg):
        with self.grasp_lock:
            if (msg.poses and self.accept_after is not None
                    and Time.from_msg(msg.header.stamp) >= self.accept_after):
                self.positions.append(msg)

    def on_bottle(self, msg):
        with self.grasp_lock:
            if self.accept_after is not None and Time.from_msg(msg.header.stamp) >= self.accept_after:
                self.bottles.append(msg)

    def on_grasp(self, msg):
        with self.grasp_lock:
            if self.accept_after is None or self.grasp is not None:
                return
            if Time.from_msg(msg.header.stamp) < self.accept_after:
                return  # cloud captured while the arm was still moving
            self.grasp = msg

    def wait_for_bottle(self):
        """(bottle positions, cylinder estimates, Contact-GraspNet candidates or None).

        All from data captured from now on: waits for `bottle_samples` positions (and, with
        use_graspnet, candidates), then up to 3 s more for at least one cylinder estimate.
        """
        with self.grasp_lock:
            self.grasp = None
            self.bottles = []
            self.positions = []
            self.accept_after = self.get_clock().now()
        self.get_logger().info('Waiting for the bottle on /bottles/positions ...')
        deadline = None
        while rclpy.ok():
            with self.grasp_lock:
                ready = (len(self.positions) >= self.bottle_samples
                         and (self.grasp is not None or not self.use_graspnet))
                if ready and deadline is None:
                    deadline = time.monotonic() + 3.0
                if deadline is not None and (self.bottles or time.monotonic() > deadline):
                    self.accept_after = None
                    return list(self.positions), list(self.bottles), self.grasp
            time.sleep(0.05)
        raise PickFailed('shutdown')

    # --------------------------------------------------------------- helpers
    @staticmethod
    def wait(future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise PickFailed(f'timed out waiting for {what}')
            time.sleep(0.01)
        return future.result()

    def sleep(self, seconds):
        """Sleep in ROS time (sim time aware)."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            time.sleep(0.05)

    def run_action(self, client, goal, timeout, what):
        handle = self.wait(client.send_goal_async(goal), 10.0, f'{what} acceptance')
        if not handle.accepted:
            raise PickFailed(f'{what} goal rejected')
        return self.wait(handle.get_result_async(), timeout, what).result

    # ----------------------------------------------------------------- arm
    def move_to_joints(self, positions, what):
        c = Constraints()
        for name, value in zip(ARM_JOINTS, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        self.plan_and_move(c, what)

    def move_to_pose(self, T, what):
        target = PoseStamped()
        target.header.frame_id = self.frame
        target.pose = matrix_to_pose(T)
        self.target_pub.publish(target)

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.frame
        pc.link_name = self.tcp_link
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        pc.constraint_region.primitive_poses.append(target.pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tcp_link
        oc.orientation = target.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = 0.03
        oc.absolute_z_axis_tolerance = 0.03
        oc.weight = 1.0
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        self.plan_and_move(c, what)

    def plan_and_move(self, constraints, what):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.vel
        req.start_state.is_diff = True
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        self.get_logger().info(f'Planning: {what}')
        result = self.run_action(self.move_group, goal, 60.0, what)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PickFailed(f'{what}: MoveIt error {result.error_code.val}')

    def move_linear(self, T_goal, what, avoid_collisions=True, loaded=False):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state.is_diff = True
        req.group_name = self.group
        req.link_name = self.tcp_link
        req.waypoints = [matrix_to_pose(T_goal)]
        req.max_step = 0.005
        # Stop the path at a joint jump (an IK branch flip), which would sweep the arm around
        req.revolute_jump_threshold = 0.5
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = self.vel * 0.5
        req.max_acceleration_scaling_factor = self.vel * 0.5
        res = self.wait(self.cartesian.call_async(req), 10.0, f'{what} path')
        if res.fraction < 0.95:
            raise PathInfeasible(
                f'{what}: only {res.fraction:.0%} of the straight path is feasible')
        self.get_logger().info(f'Moving: {what}')
        if loaded:
            self.execute_loaded(res.solution.joint_trajectory, what)
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        result = self.run_action(self.execute, goal, 60.0, what)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PickFailed(f'{what}: execution error {result.error_code.val}')

    def execute_loaded(self, trajectory, what):
        """Send a trajectory straight to the arm controller without its goal checks.

        Through MoveIt the controller's own 0.01 rad goal tolerance applies, and with the
        bottle in the gripper the sim arm sags outside it (even outside 0.05 rad) and aborts.
        A lift doesn't need to end precisely; the slip check afterwards is what matters.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp = Time().to_msg()  # zero = start now
        goal.goal_tolerance = [JointTolerance(name=n, position=-1.0, velocity=-1.0)
                               for n in trajectory.joint_names]
        goal.path_tolerance = [JointTolerance(name=n, position=-1.0, velocity=-1.0)
                               for n in trajectory.joint_names]
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()
        result = self.run_action(self.arm, goal, 60.0, what)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise PickFailed(f'{what}: arm controller error {result.error_code} '
                             f'{result.error_string}')

    def tcp_pose(self):
        tf = self.tf_buffer.lookup_transform(self.frame, self.tcp_link, Time(),
                                             timeout=Duration(seconds=1.0))
        return transform_to_matrix(tf.transform)

    def reachable(self, T):
        """Quick inverse-kinematics check (with collisions) for link_tcp at pose T."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.tcp_link
        r.avoid_collisions = True
        r.robot_state.is_diff = True
        r.pose_stamped.header.frame_id = self.frame
        r.pose_stamped.pose = matrix_to_pose(T)
        r.timeout = Duration(seconds=0.05).to_msg()
        res = self.wait(self.ik.call_async(req), 5.0, 'inverse kinematics')
        return res.error_code.val == MoveItErrorCodes.SUCCESS

    # ------------------------------------------------------------- gripper
    def move_gripper(self, position, seconds=2.0, squeeze=False):
        """Move the gripper; with squeeze=True keep pressing after reaching `position`.

        In the sim the joints follow the controller's velocity command, which is the
        trajectory's feed-forward velocity: once a trajectory reaches its last point the
        fingers just stop, so closed on a bottle they hold still without pressing and it
        slides out when lifted. With squeeze=True the trajectory continues past `position`
        slowly for another minute, so blocked fingers keep pushing at full force and empty
        ones visibly creep (which is how a slip is detected). Tolerances are erased (-1) so
        the controller doesn't abort when the fingers are blocked; the goal is left running
        until the next gripper goal replaces it. Returns where the fingers are afterwards.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        points = [(seconds, position)]
        if squeeze:
            points.append((seconds + 60.0, position + 0.3))
        for t, pos in points:
            point = JointTrajectoryPoint()
            point.positions = [float(pos)]
            point.time_from_start = Duration(seconds=t).to_msg()
            goal.trajectory.points.append(point)
        erased = JointTolerance(name=GRIPPER_JOINT, position=-1.0, velocity=-1.0)
        goal.path_tolerance = [erased]
        goal.goal_tolerance = [erased]
        handle = self.wait(self.gripper.send_goal_async(goal), 10.0, 'gripper acceptance')
        if not handle.accepted:
            raise PickFailed('gripper goal rejected')
        self.sleep(seconds + 0.5)
        return self.joints.get(GRIPPER_JOINT)

    def closed_on_something(self, position):
        return position is not None and position < self.grip_close - self.empty_margin

    def slipped(self, closed_at):
        """True if the squeezing fingers kept closing, i.e. there is nothing between them."""
        self.sleep(1.5)
        position = self.joints.get(GRIPPER_JOINT)
        return position is None or position - closed_at > self.slip_tolerance

    # --------------------------------------------------------------- scene
    def set_bottle_obstacle(self, bottle, margin=0.01):
        """Put the estimated bottle, `margin` fatter, into MoveIt's world, or remove it (None)."""
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = 'bottle'
        if bottle is None:
            obj.operation = CollisionObject.REMOVE
        else:
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[bottle.length, bottle.radius + margin]))
            R = np.eye(4)
            z = bottle.axis
            x = np.cross([0.0, 1.0, 0.0] if abs(z[1]) < 0.9 else [1.0, 0.0, 0.0], z)
            x /= np.linalg.norm(x)
            R[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            R[:3, 3] = bottle.centre
            obj.primitive_poses.append(matrix_to_pose(R))
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(obj)
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')

    def add_floor(self):
        floor = CollisionObject()
        floor.header.frame_id = self.frame
        floor.id = 'floor'
        floor.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 0.02]))
        pose = Pose()
        pose.position.z = -0.012  # top surface just below the ground
        pose.orientation.w = 1.0
        floor.primitive_poses.append(pose)
        floor.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(floor)
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')

    # ---------------------------------------------------------------- grasp
    def lookup(self, target, source, stamp):
        tf = self.tf_buffer.lookup_transform(target, source, stamp,
                                             timeout=Duration(seconds=1.0))
        return transform_to_matrix(tf.transform)

    def locate_bottle(self, positions, markers):
        """One Bottle in the planning frame from localizer positions and cylinder estimates."""
        arm = self.lookup(self.frame, 'link_base', Time())[:3, 3]
        points = []
        for msg in positions:
            try:
                T = self.lookup(self.frame, msg.header.frame_id, msg.header.stamp)
            except tf2_ros.TransformException:
                continue
            # The localizer gives one pose per detected bottle; take the one nearest the arm
            found = [(T @ pose_to_matrix(pose))[:3, 3] for pose in msg.poses]
            points.append(min(found, key=lambda q: np.linalg.norm(q[:2] - arm[:2])))
        if not points:
            raise PickFailed('no usable bottle position (TF failed)')
        position = np.median(points, axis=0)

        axes, lengths, standing_votes = [], [], []
        for m in markers:
            try:
                M = self.lookup(self.frame, m.header.frame_id, m.header.stamp) @ pose_to_matrix(m.pose)
            except tf2_ros.TransformException:
                continue
            axes.append(M[:3, 2])
            lengths.append(m.scale.z)
            standing_votes.append(abs(M[2, 2]) > 0.7)
        standing = not standing_votes or sum(standing_votes) * 2 >= len(standing_votes)
        if standing:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            ref = next(a for a, v in zip(axes, standing_votes) if not v)
            axis = np.mean([a if a @ ref >= 0 else -a for a, v in zip(axes, standing_votes)
                            if not v], axis=0)
            axis[2] = 0.0
            axis /= np.linalg.norm(axis)
        length = float(np.median(lengths)) if lengths else self.bottle_length
        centre = position.copy()
        centre[2] = length / 2.0 if standing else self.bottle_radius
        bottle = Bottle(centre, axis, self.bottle_radius, length, standing)
        self.get_logger().info(
            f'Bottle ({len(points)} positions, {len(markers)} shape estimates): '
            f'{"standing" if standing else "lying"} at x={centre[0]:.3f} y={centre[1]:.3f}, '
            f'length {100 * length:.1f} cm')
        return bottle

    @staticmethod
    def tcp_from(approach, closing, point):
        """link_tcp pose: z = approach, y = closing direction, at `point`."""
        T = np.eye(4)
        T[:3, 2] = approach / np.linalg.norm(approach)
        T[:3, 1] = closing - (closing @ T[:3, 2]) * T[:3, 2]
        T[:3, 1] /= np.linalg.norm(T[:3, 1])
        T[:3, 0] = np.cross(T[:3, 1], T[:3, 2])
        T[:3, 3] = point
        return T

    def body_grasps(self, bottle):
        """link_tcp goal poses closing across the bottle body, most preferred first."""
        down = np.array([0.0, 0.0, -1.0])
        poses = []
        if bottle.standing:
            point = bottle.centre.copy()
            point[2] = self.body_height * bottle.length
            # "Straight away from the robot": from the arm's base towards the bottle
            base = self.lookup(self.frame, 'link_base', Time())[:3, 3]
            ahead = np.array([point[0] - base[0], point[1] - base[1], 0.0])
            ahead /= np.linalg.norm(ahead)
            for yaw in self.body_yaws:
                c, s_ = math.cos(yaw), math.sin(yaw)
                horizontal = np.array([c * ahead[0] - s_ * ahead[1], s_ * ahead[0] + c * ahead[1], 0.0])
                closing = np.array([-horizontal[1], horizontal[0], 0.0])
                for tilt in self.body_tilts:
                    approach = math.cos(tilt) * horizontal + math.sin(tilt) * down
                    poses.append(self.tcp_from(approach, closing,
                                               point + self.body_depth * approach))
        else:
            closing = np.cross(down, bottle.axis)  # horizontal, across the bottle
            for tilt in self.lying_tilts:
                # Tilt the approach about the bottle axis (Rodrigues)
                k = bottle.axis
                approach = (down * math.cos(tilt) + np.cross(k, down) * math.sin(tilt)
                            + k * (k @ down) * (1 - math.cos(tilt)))
                poses.append(self.tcp_from(approach, np.cross(approach, k),
                                           bottle.centre + self.body_depth * approach))
        return poses

    def ranked_grasps(self, grasps_msg, bottle):
        """Contact-GraspNet candidates -> link_tcp goal poses, closest to the bottle axis first."""
        cam = self.lookup(self.frame, grasps_msg.header.frame_id, grasps_msg.header.stamp)
        if bottle is None:
            self.get_logger().warn('No bottle estimate; using Contact-GraspNet order')
        ranked = []
        for rank, pose in enumerate(grasps_msg.poses):
            T = cam @ pose_to_matrix(pose)
            approach = T[:3, 2]
            if approach[2] > self.max_up:
                continue  # would come from below
            fingers = T[:3, 3] + self.cgn_depth * approach
            if bottle is None:
                distance = 0.0
            else:
                d = fingers - bottle.centre
                distance = float(np.linalg.norm(d - (d @ bottle.axis) * bottle.axis))
                if distance > self.max_axis_dist:
                    continue
            tcp = np.eye(4)
            tcp[:3, :3] = T[:3, :3] @ CGN_TO_TCP
            tcp[:3, 3] = T[:3, 3] + (self.cgn_depth + self.depth_offset) * approach
            ranked.append((distance, rank, tcp))
        ranked.sort(key=lambda r: (r[0], r[1]))
        self.get_logger().info(
            f'Contact-GraspNet: {len(ranked)}/{len(grasps_msg.poses)} grasps usable; '
            'axis distances (cm): ' + ', '.join(f'{d * 100:.1f}' for d, _, _ in ranked))
        return [tcp for _, _, tcp in ranked]

    def reachable_poses(self, poses, back_off, limit):
        """Poses (either finger orientation) whose grasp and pre-grasp both pass IK."""
        found = []
        for tcp in poses:
            for turned in (False, True):
                goal = tcp.copy()
                if turned:
                    goal[:3, :3] = tcp[:3, :3] @ HALF_TURN_Z
                pre = goal.copy()
                pre[:3, 3] -= back_off * goal[:3, 2]
                if self.reachable(pre) and self.reachable(goal):
                    found.append(goal)
                    break
            if len(found) >= limit:
                break
        return found

    def reach(self, bottle, grasps_msg):
        """Move to a pre-grasp and straight in to the grasp; returns (pre-grasp, approach)."""
        back_off = self.pre_dist + bottle.radius
        # With the bottle (barely fatter) in MoveIt's world, so poses where the gripper's palm
        # or knuckles would hit it are rejected, not just unreachable ones
        self.set_bottle_obstacle(bottle, margin=0.003)
        try:
            poses = self.reachable_poses(self.body_grasps(bottle), back_off, self.max_candidates)
        finally:
            self.set_bottle_obstacle(None)
        self.get_logger().info(f'Body grasp: {len(poses)} reachable pose(s)')
        options = [('body grasp', poses, back_off)]
        if self.use_graspnet and grasps_msg is not None:
            options.append(('Contact-GraspNet grasp',
                            self.ranked_grasps(grasps_msg, bottle)[:self.max_candidates],
                            self.pre_dist))

        self.move_gripper(self.grip_open)
        self.set_bottle_obstacle(bottle)  # plan around it to the pre-grasp
        try:
            for name, poses, back_off in options:
                for n, tcp in enumerate(poses, 1):
                    approach = tcp[:3, 2]
                    self.get_logger().info(
                        f'{name} {n}: x={tcp[0, 3]:.3f} y={tcp[1, 3]:.3f} z={tcp[2, 3]:.3f}, '
                        f'approach ({approach[0]:.2f}, {approach[1]:.2f}, {approach[2]:.2f})')
                    if self.move_to_pre_grasp(tcp, back_off):
                        break
                else:
                    continue
                break
            else:
                raise PickFailed('MoveIt could not reach any of the grasps')
        finally:
            self.set_bottle_obstacle(None)  # the fingers are about to go around it

        pre = tcp.copy()
        pre[:3, 3] -= back_off * approach
        try:
            self.move_linear(tcp, 'approach')
        except PickFailed:
            # Back straight out: a planned move from near the bottle could sweep through it
            self.move_linear(pre, 'retreat', avoid_collisions=False)
            raise
        return pre, approach

    def grasp_and_lift(self, pre, approach):
        closed_at = self.move_gripper(self.grip_close, squeeze=True)
        if not self.closed_on_something(closed_at):
            self.move_gripper(self.grip_open)
            self.move_linear(pre, 'retreat', avoid_collisions=False)
            raise PickFailed(f'gripper closed on nothing (drive_joint = {closed_at:.3f})')
        self.get_logger().info(f'Gripper closed on the bottle (drive_joint = {closed_at:.3f})')

        # No collision checks: the fingers may be touching the floor
        start = self.tcp_pose()
        lifted = start.copy()
        lifted[2, 3] += self.lift
        try:
            self.move_linear(lifted, 'lift', avoid_collisions=False, loaded=True)
        except PathInfeasible as err:
            self.get_logger().info(f'{err}; lifting back along the approach instead')
            lifted = start.copy()
            lifted[:3, 3] -= self.lift * approach
            self.move_linear(lifted, 'lift', avoid_collisions=False, loaded=True)
        if self.slipped(closed_at):
            raise PickFailed('the bottle slipped out while lifting')

    def move_to_pre_grasp(self, tcp, back_off):
        """Plan to `back_off` before `tcp` along its approach, also with the gripper turned
        180 deg; True if reached. `tcp` is updated in place to the orientation that worked.
        """
        for turned in (False, True):
            goal = tcp.copy()
            if turned:
                goal[:3, :3] = tcp[:3, :3] @ HALF_TURN_Z
            pre = goal.copy()
            pre[:3, 3] -= back_off * goal[:3, 2]
            try:
                self.move_to_pose(pre, 'pre-grasp')
                tcp[:3, :3] = goal[:3, :3]
                return True
            except PickFailed as err:
                self.get_logger().warn(str(err))
        return False

    # ---------------------------------------------------------------- main
    def run(self):
        log = self.get_logger()
        try:
            for client, name in [(self.move_group, '/move_action'),
                                 (self.execute, '/execute_trajectory'),
                                 (self.arm, 'arm controller'),
                                 (self.gripper, 'gripper controller')]:
                log.info(f'Waiting for {name} ...')
                while rclpy.ok() and not client.wait_for_server(timeout_sec=2.0):
                    log.info(f'... still waiting for {name}', throttle_duration_sec=10.0)
            for client in (self.cartesian, self.apply_scene, self.ik):
                while rclpy.ok() and not client.wait_for_service(timeout_sec=2.0):
                    pass
            self.add_floor()

            for attempt in range(1, self.max_attempts + 1):
                try:
                    self.move_gripper(self.grip_open)
                    self.move_to_joints(self.view_joints, 'pick view pose')
                    self.sleep(self.settle_time)
                    positions, shapes, grasps = self.wait_for_bottle()
                    log.info(f'Attempt {attempt}/{self.max_attempts}: reaching for the bottle')
                    bottle = self.locate_bottle(positions, shapes)
                    pre, approach = self.reach(bottle, grasps)
                    self.grasp_and_lift(pre, approach)
                    log.info('Bottle grasped and lifted.')
                    return
                except PickFailed as err:
                    log.warn(f'Attempt {attempt} failed: {err}')
                    # Never leave a squeeze running: pressing past the finger limits on a
                    # dropped bottle makes the sim's contact physics blow up
                    try:
                        self.move_gripper(self.grip_open)
                    except PickFailed:
                        pass
            log.error(f'Giving up after {self.max_attempts} attempts.')
        except PickFailed as err:
            log.error(str(err))
        except Exception as err:  # noqa: BLE001 - keep the node alive to report it
            if rclpy.ok():
                log.error(f'Unexpected error: {err!r}')


def main(args=None):
    rclpy.init(args=args)
    node = PickBottle()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
