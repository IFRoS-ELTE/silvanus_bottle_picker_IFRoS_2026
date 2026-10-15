"""Grasp a bottle with the xArm6 using the grasps from grasp_inference, and lift it.

    ros2 run grasp_pipeline pick_bottle --ros-args -p use_sim_time:=true

Needs move_group (ros2 launch grasp_pipeline moveit_sim.launch.py) and the sim's
xarm6 / xarm_gripper trajectory controllers.

On the real robot (silvanus-up.sh --arm: its own move_group and the AG-95 gripper), with the
parameters ROBOT=1 PICK=1 ./sim/grasp_demo.sh passes (see the README):

    ros2 run grasp_pipeline pick_bottle --ros-args -p use_view_pose:=false \
      -p tcp_link:=link_eef -p tcp_offset:='[0.0, 0.0, 0.19, 0.0, 0.0, 0.0]' \
      -p gripper_interface:=gripper_command \
      -p gripper_action:=/dh_ag95_gripper_controller/gripper_cmd \
      -p gripper_joint:=left_outer_knuckle_joint -p gripper_close:=0.93

Sequence:
  1. Open the gripper and move the arm to the pick view pose (camera looking at the ground
     just in front of the robot, the only area the arm can reach). With use_view_pose:=false
     the arm stays where it is: the camera must already see the bottle.
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
  7. Only with drop_joints_deg set: carry the bottle to that arm pose (on the robot, over
     the box on its back) and open the gripper there.
  8. Only with home_joints_deg set: back to the home pose. The arm also goes there at the
     start of every attempt (before step 1's view pose), so at the very beginning and
     after a failed attempt.
If a step fails it returns to the home / view pose and waits for new grasps, up to
`max_attempts` times. With drop_joints_deg set and continuous:=true (the default), after
step 8 it starts again at step 1 for the next bottle, until stopped.

Contact-GraspNet grasp frame: z = approach, x = finger closing direction, and the point
between the fingers is `cgn_gripper_depth` along z from the origin.
xArm gripper link_tcp: z = approach, fingers close along y.
"""

from collections import namedtuple
import math
import re
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from grasp_pipeline.geometry import matrix_to_pose, pose_to_matrix, transform_to_matrix

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Contact-GraspNet closes along x, the xArm gripper along y: rotate -90 deg about z
CGN_TO_TCP = np.array([[0.0, 1.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])
# A parallel gripper is symmetric under a half turn about the approach axis
HALF_TURN_Z = np.diag([-1.0, -1.0, 1.0])


def xyz_rpy_to_matrix(xyz, rpy):
    """4x4 transform from a translation and URDF-style roll, pitch, yaw."""
    (cr, cp, cy), (sr, sp, sy) = np.cos(rpy), np.sin(rpy)
    T = np.eye(4)
    T[:3, :3] = (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
                 @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
                 @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))
    T[:3, 3] = xyz
    return T


# A bottle estimate in the planning frame; `support` is the height of the surface it rests on
Bottle = namedtuple('Bottle', 'centre axis radius length standing support', defaults=(0.0,))


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
        # Gripper TCP (z = approach, fingers close along y, origin between the fingertips) as
        # [x, y, z, roll, pitch, yaw] in tcp_link. Identity for the sim's link_tcp. The robot
        # has no link_tcp: 0.19 m along link_eef's z (the URDF's grasp_link). Its real AG-95
        # closes along link_eef's y, like link_tcp, so no rotation (its URDF mount has it 90 deg
        # off, along x).
        self.declare_parameter('tcp_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # false: never move to a view pose; grasp from wherever the arm is as soon as the bottle
        # is seen (the real robot, where the sim's view_joints were never checked)
        self.declare_parameter('use_view_pose', True)
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
        # Time MoveIt's IK gets per candidate pose (s). Near the edge of the arm's reach it
        # needs more random restarts to find a solution, so too short rejects reachable poses
        self.declare_parameter('ik_timeout', 0.05)
        # true: classify every candidate (for the log and /grasp/body_candidates) instead of
        # stopping at max_candidates reachable ones. Slower: every pose gets its IK checks
        self.declare_parameter('check_all_candidates', False)
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
        # 'trajectory': FollowJointTrajectory (the sim's xArm gripper), 'gripper_command':
        # control_msgs/GripperCommand (the robot's AG-95)
        self.declare_parameter('gripper_interface', 'trajectory')
        self.declare_parameter('gripper_action',
                               '/xarm_gripper_traj_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_joint', 'drive_joint')
        self.declare_parameter('gripper_max_effort', 0.0)    # GripperCommand only; 0 = default
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_close', 0.85)
        # Fingers stopping this close to fully closed = nothing (or just a fingertip) in between
        self.declare_parameter('gripper_empty_margin', 0.1)
        # Squeezing fingers creeping more than this after the lift = the bottle slipped out
        self.declare_parameter('slip_tolerance', 0.004)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('max_attempts', 30)           # whole pick attempts
        # After the lift, carry the bottle to this arm pose (joint1..joint6, degrees) and open
        # the gripper there: on the robot, over the box on its back. Empty: no drop, the arm
        # stays holding the bottle
        # (Declared by type: an empty-list default would make it a byte array)
        self.declare_parameter('drop_joints_deg', Parameter.Type.DOUBLE_ARRAY)
        # Home arm pose (joint1..joint6, degrees): at the start of every attempt (so also
        # before retrying a failed one) and after the drop. Empty: the arm isn't sent home
        self.declare_parameter('home_joints_deg', Parameter.Type.DOUBLE_ARRAY)
        # With drop_joints_deg set: after dropping a bottle and going home, wait for the next
        # one and pick it too, until stopped (or max_attempts fail in a row on one bottle)
        self.declare_parameter('continuous', True)
        # Links allowed to touch the carried bottle: everything mounted below this one (the
        # gripper, and the camera next to it), from /robot_description
        self.declare_parameter('gripper_root_link', 'link_eef')
        # Plans MoveIt makes per motion request (run in parallel within planning_time; the
        # shortest valid one is used)
        self.declare_parameter('planning_attempts', 30)
        self.declare_parameter('planning_time', 5.0)         # s per motion request

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame = p('planning_frame')
        self.group = p('group')
        self.tcp_link = p('tcp_link')
        offset = [float(v) for v in p('tcp_offset')]
        if len(offset) != 6:
            raise ValueError('tcp_offset must be [x, y, z, roll, pitch, yaw]')
        self.tcp_offset = xyz_rpy_to_matrix(offset[:3], offset[3:])
        self.tcp_offset_inv = np.linalg.inv(self.tcp_offset)
        self.use_view_pose = bool(p('use_view_pose'))
        self.view_joints = list(p('view_joints'))
        self.settle_time = float(p('settle_time'))
        self.cgn_depth = float(p('cgn_gripper_depth'))
        self.depth_offset = float(p('grasp_depth_offset'))
        self.pre_dist = float(p('pre_grasp_distance'))
        self.max_up = float(p('max_approach_up'))
        self.max_axis_dist = float(p('max_axis_distance'))
        self.max_candidates = int(p('max_candidates'))
        self.ik_timeout = float(p('ik_timeout'))
        self.check_all = bool(p('check_all_candidates'))
        self.use_graspnet = bool(p('use_graspnet'))
        self.bottle_radius = float(p('bottle_radius'))
        self.bottle_length = float(p('bottle_length'))
        self.body_height = float(p('body_grasp_height'))
        self.body_depth = float(p('body_grasp_depth'))
        self.body_yaws = [math.radians(v) for v in p('body_yaws')]
        self.body_tilts = [math.radians(v) for v in p('body_tilts')]
        self.lying_tilts = [math.radians(v) for v in p('lying_tilts')]
        self.bottle_samples = int(p('bottle_samples'))
        self.gripper_interface = p('gripper_interface')
        if self.gripper_interface not in ('trajectory', 'gripper_command'):
            raise ValueError("gripper_interface must be 'trajectory' or 'gripper_command'")
        self.gripper_joint = p('gripper_joint')
        self.grip_effort = float(p('gripper_max_effort'))
        self.grip_open = float(p('gripper_open'))
        self.grip_close = float(p('gripper_close'))
        self.empty_margin = float(p('gripper_empty_margin'))
        self.slip_tolerance = float(p('slip_tolerance'))
        self.lift = float(p('lift_height'))
        self.vel = float(p('velocity_scaling'))
        self.max_attempts = int(p('max_attempts'))
        self.drop_joints = self.joints_param('drop_joints_deg')
        self.home_joints = self.joints_param('home_joints_deg')
        self.continuous = bool(p('continuous'))
        self.gripper_root = p('gripper_root_link')
        self.touch_links = []
        self.bottle_attached = False
        # The robot model, for the links that may touch the carried bottle (published latched)
        self.create_subscription(
            String, '/robot_description', self.on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.planning_attempts = int(p('planning_attempts'))
        self.planning_time = float(p('planning_time'))

        cb = ReentrantCallbackGroup()
        self.move_group = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory',
                                    callback_group=cb)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/xarm6_traj_controller/follow_joint_trajectory',
                                callback_group=cb)
        self.gripper = ActionClient(
            self, GripperCommand if self.gripper_interface == 'gripper_command'
            else FollowJointTrajectory, p('gripper_action'), callback_group=cb)
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path',
                                            callback_group=cb)
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                              callback_group=cb)
        self.ik = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joints = {}
        self.joints_stamp = None     # header stamp of the latest /joint_states (robot clock)
        self.joints_received = None  # time.monotonic() when it arrived
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
        # Body-grasp candidates and why they were rejected, for RViz (see show_candidates)
        self.candidates_pub = self.create_publisher(MarkerArray, '/grasp/body_candidates', 10)

        self.grasp_lock = threading.Lock()
        self.accept_after = None  # only grasps from clouds newer than this are used
        self.grasp = None

        threading.Thread(target=self.run, daemon=True).start()

    # ---------------------------------------------------------------- inputs
    def joints_param(self, name):
        """An optional joint1..joint6 parameter in degrees -> radians ([] if not given)."""
        try:
            values = self.get_parameter(name).value
        except ParameterUninitializedException:  # not given
            values = []
        joints = [math.radians(v) for v in values]
        if joints and len(joints) != len(ARM_JOINTS):
            raise ValueError(f'{name} needs {len(ARM_JOINTS)} values (joint1..joint6)')
        return joints

    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            self.joints_stamp = Time.from_msg(msg.header.stamp)
            self.joints_received = time.monotonic()

    def on_robot_description(self, msg):
        """Collect gripper_root_link and every link mounted below it as touch_links."""
        urdf = re.sub(r'<!--.*?-->', '', msg.data, flags=re.S)
        children = {}
        for joint in re.findall(r'<joint\b.*?</joint>', urdf, flags=re.S):
            parent = re.search(r'<parent\s+link="([^"]+)"', joint)
            child = re.search(r'<child\s+link="([^"]+)"', joint)
            if parent and child:
                children.setdefault(parent.group(1), []).append(child.group(1))
        links, todo = [], [self.gripper_root]
        while todo:
            link = todo.pop()
            links.append(link)
            todo.extend(children.get(link, []))
        self.touch_links = links

    def joints_fresh(self, max_age=1.0):
        return (self.joints_received is not None
                and time.monotonic() - self.joints_received < max_age)

    def robot_now(self):
        """Return the current time on the clock that stamps the robot's data.

        The camera, TF and joint states are stamped by the machine they run on. On the real
        robot that is the robot computer, whose clock can be far off this laptop's (it has no
        internet to sync to), so get_clock().now() can't be compared with their stamps. The
        latest /joint_states stamp can. Falls back to this node's clock (sim time in the sim).
        """
        if self.joints_fresh() and self.joints_stamp is not None:
            return self.joints_stamp
        return self.get_clock().now()

    def wait_for_arm(self, timeout=30.0):
        """Wait for live /joint_states, i.e. the arm controllers running.

        On the robot the xArm driver switches every controller off (arm, gripper and
        joint_state_broadcaster) when the arm reports an error, is paused or stopped, or
        leaves servo mode, and on again once it is ready. Without them no goal ever finishes.
        """
        deadline = time.monotonic() + timeout
        if not self.joints_fresh():
            self.get_logger().warn('No live /joint_states: the arm controllers are not running. '
                                   'Waiting for them ...')
        while rclpy.ok() and not self.joints_fresh():
            if time.monotonic() > deadline:
                raise PickFailed(
                    f'no /joint_states for {timeout:.0f} s: the arm controllers are inactive. '
                    'Check `ros2 control list_controllers` and /xarm/robot_states (err, state, '
                    'mode) on the robot')
            time.sleep(0.1)

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
            self.accept_after = self.robot_now()
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

        link_pose = matrix_to_pose(T @ self.tcp_offset_inv)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.frame
        pc.link_name = self.tcp_link
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        pc.constraint_region.primitive_poses.append(link_pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tcp_link
        oc.orientation = link_pose.orientation
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
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
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
        req.waypoints = [matrix_to_pose(T_goal @ self.tcp_offset_inv)]
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
        return transform_to_matrix(tf.transform) @ self.tcp_offset

    def reachable(self, T, avoid_collisions=True):
        """Quick inverse-kinematics check (with collisions) for the gripper TCP at pose T."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.tcp_link
        r.avoid_collisions = avoid_collisions
        r.robot_state.is_diff = True
        r.pose_stamped.header.frame_id = self.frame
        r.pose_stamped.pose = matrix_to_pose(T @ self.tcp_offset_inv)
        r.timeout = Duration(seconds=self.ik_timeout).to_msg()
        res = self.wait(self.ik.call_async(req), 5.0 + self.ik_timeout, 'inverse kinematics')
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

        With gripper_interface 'gripper_command' (the AG-95) the gripper's own position
        controller keeps pressing on whatever stopped the fingers, so `squeeze` isn't needed.
        """
        if self.gripper_interface == 'gripper_command':
            return self.command_gripper(position, seconds)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_joint]
        points = [(seconds, position)]
        if squeeze:
            points.append((seconds + 60.0, position + 0.3))
        for t, pos in points:
            point = JointTrajectoryPoint()
            point.positions = [float(pos)]
            point.time_from_start = Duration(seconds=t).to_msg()
            goal.trajectory.points.append(point)
        erased = JointTolerance(name=self.gripper_joint, position=-1.0, velocity=-1.0)
        goal.path_tolerance = [erased]
        goal.goal_tolerance = [erased]
        handle = self.wait(self.gripper.send_goal_async(goal), 10.0, 'gripper acceptance')
        if not handle.accepted:
            raise PickFailed('gripper goal rejected')
        self.sleep(seconds + 0.5)
        return self.joints.get(self.gripper_joint)

    def command_gripper(self, position, seconds):
        """GripperCommand goal; done when the fingers reach `position` or stall on something."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.grip_effort
        result = self.run_action(self.gripper, goal, seconds + 10.0, 'gripper')
        self.sleep(0.5)
        return self.joints.get(self.gripper_joint, result.position)

    def closed_on_something(self, position):
        return position is not None and position < self.grip_close - self.empty_margin

    def slipped(self, closed_at):
        """True if the squeezing fingers kept closing, i.e. there is nothing between them."""
        self.sleep(1.5)
        position = self.joints.get(self.gripper_joint)
        return position is None or position - closed_at > self.slip_tolerance

    # --------------------------------------------------------------- scene
    def bottle_object(self, bottle, margin=0.0):
        """The bottle, `margin` fatter, as a MoveIt collision object (or its removal: None)."""
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = 'bottle'
        if bottle is None:
            obj.operation = CollisionObject.REMOVE
            return obj
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
        return obj

    def set_bottle_obstacle(self, bottle, margin=0.01):
        """Put the estimated bottle, `margin` fatter, into MoveIt's world, or remove it (None)."""
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(self.bottle_object(bottle, margin))
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')

    def attach_bottle(self, bottle):
        """Attach the grasped bottle to the gripper in MoveIt's model, or detach it (None).

        Attached, it moves with the arm and planning keeps it clear of the robot's body; the
        gripper's links (touch_links) may touch it. Detaching also removes it from the world.
        """
        if bottle is not None and not self.touch_links:
            self.get_logger().warn(
                'No /robot_description received: the attached bottle may not touch the gripper, '
                'so MoveIt will see the grasp itself as a collision (error -26)')
        aco = AttachedCollisionObject(link_name=self.tcp_link)
        aco.object = self.bottle_object(bottle)
        aco.touch_links = list(self.touch_links)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        if bottle is None:
            scene.world.collision_objects.append(self.bottle_object(None))
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')
        self.bottle_attached = bottle is not None

    def set_support(self, bottle, size=0.3):
        """Put the surface the bottle rests on into MoveIt's world, if it isn't the floor.

        Its extent isn't measured, so a `size` square under the bottle: enough to keep the
        gripper from being planned through the table top around it.
        """
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = 'support'
        if bottle.support < 0.03:  # the floor, already in the world
            obj.operation = CollisionObject.REMOVE
        else:
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                                                 dimensions=[size, size, 0.02]))
            pose = Pose()
            pose.position.x, pose.position.y = (float(v) for v in bottle.centre[:2])
            pose.position.z = float(bottle.support) - 0.013  # top just below the surface
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(obj)
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')

    def clear_leftovers(self):
        """Remove a bottle and table top an earlier run left in move_group's scene.

        move_group outlives this node (on the robot it is part of the robot's bringup), so a run
        stopped while carrying a bottle leaves it attached to the gripper: every plan after that
        starts in collision (MoveIt error -26, START_STATE_INVALID).
        """
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name=self.tcp_link)
        aco.object.id = 'bottle'
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(aco)
        for name in ('bottle', 'support'):
            obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
            obj.header.frame_id = self.frame
            scene.world.collision_objects.append(obj)
        self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                  10.0, 'planning scene')
        self.bottle_attached = False

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

        axes, lengths, standing_votes, supports = [], [], [], []
        for m in markers:
            try:
                M = self.lookup(self.frame, m.header.frame_id, m.header.stamp) @ pose_to_matrix(m.pose)
            except tf2_ros.TransformException:
                continue
            axes.append(M[:3, 2])
            lengths.append(m.scale.z)
            standing_votes.append(abs(M[2, 2]) > 0.7)
            # The surface it rests on (floor or table): below the centre by half the length
            # (standing) or the radius (lying)
            supports.append(M[2, 3] - (m.scale.z if standing_votes[-1] else m.scale.x) / 2.0)
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
        # Without a shape estimate, assume the floor
        support = max(0.0, float(np.median(supports))) if supports else 0.0
        centre = position.copy()
        centre[2] = support + (length / 2.0 if standing else self.bottle_radius)
        bottle = Bottle(centre, axis, self.bottle_radius, length, standing, support)
        self.get_logger().info(
            f'Bottle ({len(points)} positions, {len(markers)} shape estimates): '
            f'{"standing" if standing else "lying"} at x={centre[0]:.3f} y={centre[1]:.3f} '
            f'z={centre[2]:.3f}, length {100 * length:.1f} cm, on a surface at '
            f'z={support:.3f}')
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
            point[2] = bottle.support + self.body_height * bottle.length
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
        """Poses (either finger orientation) whose grasp and pre-grasp both pass IK.

        Also returns why each candidate was kept or rejected, one per pose in `poses`:
        'reachable', 'pre-grasp' (the grasp is reachable but not the pose back_off before it),
        'collision' (reachable only by going through something), 'out of reach', or
        'not checked' (after `limit` reachable ones were found, unless check_all_candidates).
        """
        found, status = [], []
        for tcp in poses:
            if len(found) >= limit and not self.check_all:
                status.append('not checked')
                continue
            why = None
            for turned in (False, True):
                goal = tcp.copy()
                if turned:
                    goal[:3, :3] = tcp[:3, :3] @ HALF_TURN_Z
                pre = goal.copy()
                pre[:3, 3] -= back_off * goal[:3, 2]
                if self.reachable(goal):
                    if self.reachable(pre):
                        found.append(goal)
                        why = 'reachable'
                        break
                    why = 'pre-grasp'
                elif why is None and self.reachable(goal, avoid_collisions=False):
                    why = 'collision'
            status.append(why or 'out of reach')
        return found[:limit], status

    def log_candidates(self, bottle, poses, status, back_off):
        """Log every body-grasp candidate: where the fingers and the wrist would be, and why.

        yaw: approach direction around the bottle, 0 = straight away from the arm's base,
        positive = counter-clockwise seen from above. tilt: below horizontal. Positions in
        the planning frame. wrist: link_eef (tcp_offset back from the fingertips), with its
        distance from link_base, which is what the arm's reach limits.
        """
        base = self.lookup(self.frame, 'link_base', Time())[:3, 3]
        ahead = bottle.centre[:2] - base[:2]
        ahead /= np.linalg.norm(ahead)
        log = self.get_logger()
        log.info(f'{len(poses)} candidate grasp(s); link_base at '
                 f'({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}):')
        for i, (tcp, why) in enumerate(zip(poses, status), 1):
            a = tcp[:3, 2]
            tilt = round(math.degrees(math.asin(max(-1.0, min(1.0, -a[2]))))) + 0  # no -0
            yaw = round(math.degrees(math.atan2(ahead[0] * a[1] - ahead[1] * a[0],
                                                ahead @ a[:2]))) + 0
            p, pre = tcp[:3, 3], tcp[:3, 3] - back_off * a
            wrist = (tcp @ self.tcp_offset_inv)[:3, 3]
            log.info(
                f'  {i:2d} {why:12s} yaw {yaw:+5.0f} tilt {tilt:3.0f} | fingers '
                f'({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) pre-grasp ({pre[0]:.3f}, {pre[1]:.3f}, '
                f'{pre[2]:.3f}) wrist ({wrist[0]:.3f}, {wrist[1]:.3f}, {wrist[2]:.3f}) '
                f'{np.linalg.norm(wrist - base):.3f} m from link_base')

    CANDIDATE_COLOURS = {'reachable': (0.1, 0.9, 0.1), 'pre-grasp': (1.0, 0.9, 0.0),
                         'collision': (1.0, 0.5, 0.0), 'out of reach': (0.9, 0.1, 0.1),
                         'not checked': (0.6, 0.6, 0.6)}

    def show_candidates(self, bottle, poses, status, back_off):
        """Publish the bottle and each grasp candidate on /grasp/body_candidates for RViz.

        One arrow per candidate from its pre-grasp to the point between the fingertips (along
        the approach), coloured by `status`: green reachable, yellow pre-grasp unreachable,
        orange in collision, red out of reach, grey not checked. The bottle is the estimate
        planned around (blue), so it can be compared with the real one in the camera's cloud.
        """
        markers = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = self.frame
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()

        def marker(ns, mid, mtype):
            m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD)
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.pose.orientation.w = 1.0
            return m

        cyl = marker('bottle', 0, Marker.CYLINDER)
        T = np.eye(4)
        z = bottle.axis
        x = np.cross([0.0, 1.0, 0.0] if abs(z[1]) < 0.9 else [1.0, 0.0, 0.0], z)
        x /= np.linalg.norm(x)
        T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        T[:3, 3] = bottle.centre
        cyl.pose = matrix_to_pose(T)
        cyl.scale.x = cyl.scale.y = 2.0 * bottle.radius
        cyl.scale.z = bottle.length
        cyl.color.b, cyl.color.g, cyl.color.a = 1.0, 0.4, 0.4
        markers.markers.append(cyl)

        for i, (tcp, why) in enumerate(zip(poses, status)):
            arrow = marker('candidates', i, Marker.ARROW)
            tip = tcp[:3, 3]
            arrow.points = [Point(x=float(a), y=float(b), z=float(c))
                            for a, b, c in (tip - back_off * tcp[:3, 2], tip)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.004, 0.01, 0.015
            arrow.color.r, arrow.color.g, arrow.color.b = self.CANDIDATE_COLOURS[why]
            arrow.color.a = 1.0 if why == 'reachable' else 0.6
            markers.markers.append(arrow)

        counts = {why: status.count(why) for why in self.CANDIDATE_COLOURS if why in status}
        base = self.lookup(self.frame, 'link_base', Time())[:3, 3]
        reach = float(np.hypot(*(bottle.centre[:2] - base[:2])))
        text = marker('summary', 0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y = (float(v) for v in bottle.centre[:2])
        text.pose.position.z = float(bottle.centre[2] + bottle.length / 2.0 + 0.08)
        text.scale.z = 0.03
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = (f'{reach:.2f} m from link_base\n'
                     + ', '.join(f'{n} {why}' for why, n in counts.items()))
        markers.markers.append(text)
        self.candidates_pub.publish(markers)
        return counts, reach

    def reach(self, bottle, grasps_msg):
        """Move to a pre-grasp and straight in to the grasp; returns (pre-grasp, approach)."""
        back_off = self.pre_dist + bottle.radius
        self.set_support(bottle)
        # With the bottle (barely fatter) in MoveIt's world, so poses where the gripper's palm
        # or knuckles would hit it are rejected, not just unreachable ones
        self.set_bottle_obstacle(bottle, margin=0.003)
        candidates = self.body_grasps(bottle)
        try:
            poses, status = self.reachable_poses(candidates, back_off, self.max_candidates)
        finally:
            self.set_bottle_obstacle(None)
        counts, reach = self.show_candidates(bottle, candidates, status, back_off)
        self.log_candidates(bottle, candidates, status, back_off)
        self.get_logger().info(
            f'Body grasp: {len(poses)} reachable pose(s) of {len(candidates)} '
            f'({", ".join(f"{n} {why}" for why, n in counts.items())}); the bottle is '
            f'{reach:.2f} m from link_base horizontally. See /grasp/body_candidates in RViz')
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

    def grasp_and_lift(self, bottle, pre, approach):
        closed_at = self.move_gripper(self.grip_close, squeeze=True)
        if not self.closed_on_something(closed_at):
            self.move_gripper(self.grip_open)
            self.move_linear(pre, 'retreat', avoid_collisions=False)
            raise PickFailed(f'gripper closed on nothing ({self.gripper_joint} = {closed_at:.3f})')
        self.get_logger().info(
            f'Gripper closed on the bottle ({self.gripper_joint} = {closed_at:.3f})')
        if self.drop_joints:
            self.attach_bottle(bottle)  # carried to the drop pose: plan around it

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

    def drop(self):
        """Carry the grasped bottle to drop_joints_deg and let go of it there."""
        self.move_to_joints(self.drop_joints, 'drop pose')
        self.move_gripper(self.grip_open)
        self.attach_bottle(None)

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

    def go_home(self):
        """Back to home_joints_deg, if set; a failure is only logged."""
        if not self.home_joints:
            return
        try:
            self.wait_for_arm()
            self.move_to_joints(self.home_joints, 'home pose')
            self.get_logger().info('Arm back at the home pose.')
        except PickFailed as err:
            self.get_logger().error(f'Could not move back to the home pose: {err}')

    # ---------------------------------------------------------------- main
    def pick_one(self):
        """Wait for a bottle and grasp and lift it, up to max_attempts tries; True if lifted."""
        log = self.get_logger()
        for attempt in range(1, self.max_attempts + 1):
            try:
                self.wait_for_arm()
                self.move_gripper(self.grip_open)
                if self.home_joints:
                    self.move_to_joints(self.home_joints, 'home pose')
                    self.sleep(self.settle_time)
                if self.use_view_pose:
                    self.move_to_joints(self.view_joints, 'pick view pose')
                    self.sleep(self.settle_time)
                positions, shapes, grasps = self.wait_for_bottle()
                log.info(f'Attempt {attempt}/{self.max_attempts}: reaching for the bottle')
                bottle = self.locate_bottle(positions, shapes)
                pre, approach = self.reach(bottle, grasps)
                self.grasp_and_lift(bottle, pre, approach)
                log.info('Bottle grasped and lifted.')
                return True
            except PickFailed as err:
                log.warn(f'Attempt {attempt} failed: {err}')
                if self.bottle_attached:
                    try:
                        self.attach_bottle(None)
                    except PickFailed:
                        pass
                # Never leave a squeeze running: pressing past the finger limits on a
                # dropped bottle makes the sim's contact physics blow up
                try:
                    if self.joints_fresh():  # with the controllers off it would time out
                        self.move_gripper(self.grip_open)
                except PickFailed:
                    pass
        log.error(f'Giving up after {self.max_attempts} attempts.')
        self.go_home()
        return False

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
            self.clear_leftovers()
            self.add_floor()
            if not self.use_view_pose:
                log.info('No view pose: grasping from where the arm is as soon as the bottle '
                         'is seen')

            picked = 0
            while rclpy.ok():
                if not self.pick_one():
                    return
                picked += 1
                if not self.drop_joints:
                    return  # still holding the bottle
                try:
                    self.drop()
                    log.info(f'Bottle dropped at the drop pose ({picked} so far).')
                except PickFailed as err:
                    # Not retried: the bottle is still in the gripper
                    log.error(f'Could not drop the bottle: {err}. Still holding it.')
                    return
                self.go_home()
                if not self.continuous:
                    return
                log.info('Looking for the next bottle ...')
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
