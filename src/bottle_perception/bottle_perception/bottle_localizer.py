"""3D Bottle Localizer.

Combines three things to know WHERE each bottle is in the world:
  1. YOLO bounding box (pixel u, v)            <- /bottles/detections
  2. Distance at that pixel (depth)            <- /camera/depth/image_raw
  3. Camera lens parameters (fx, fy, cx, cy)  <- /camera/color/camera_info
and then uses TF to transform the point from the camera frame to the world frame.

    Outputs: /bottles/positions  geometry_msgs/PoseArray        (one pose per bottle)
             /bottles/markers    visualization_msgs/MarkerArray (cylinders for RViz)

Assumption v1: depth ALIGNED with color. In Gazebo this is true.
On a real D435, you'll need to launch the driver with align_depth.enable:=true.
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (teaches TF how to transform PointStamped)


class BottleLocalizer(Node):
    def __init__(self) -> None:
        super().__init__('bottle_localizer')

        # --- Parameters (can be changed with -p name:=value) -----------------
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_frame', 'odom')    # frame in which we publish
        self.declare_parameter('roi_fraction', 0.3)        # central part of the bounding box we look at
        self.declare_parameter('max_depth', 4.0)          # ignore readings further than this (m)
        self.declare_parameter('bottle_radius', 0.035)    # camera sees the surface, not the center axis

        self.target_frame = self.get_parameter('target_frame').value
        self.roi = float(self.get_parameter('roi_fraction').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.radius = float(self.get_parameter('bottle_radius').value)

        # --- State: the latest data received -------------------------
        self.bridge = CvBridge()
        self.K = None             # (fx, fy, cx, cy) of the color camera
        self.info_size = None     # (width, height) of the color image
        self.depth = None         # latest depth image, in METERS

        # --- TF: where each part of the robot is relative to the others -----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Inputs and outputs -------------------------------------------
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/bottles/detections',
                                 self.on_detections, 10)
        self.pose_pub = self.create_publisher(PoseArray, '/bottles/positions', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bottles/markers', 10)

        self.get_logger().info(f'publishing positions in frame "{self.target_frame}"')

    # 1) Lens parameters: just save them
    def on_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.get_logger().info(f'camera_info received: {msg.width}x{msg.height}')
        # K is a 3x3 matrix stored as a list of 9: [fx 0 cx, 0 fy cy, 0 0 1]
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.info_size = (msg.width, msg.height)

    # 2) Depth: save the latest one, always in meters
    def on_depth(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == '16UC1':          # real camera: integer millimeters
            depth = depth.astype(np.float32) / 1000.0
        else:                                # Gazebo: 32FC1, already in meters
            depth = depth.astype(np.float32)
        self.depth = depth

    # 3) Every time YOLO detects something, calculate its 3D position
    def on_detections(self, msg: Detection2DArray) -> None:
        if self.K is None or self.depth is None:
            self.get_logger().warn('waiting for camera_info and depth...',
                                   throttle_duration_sec=2.0)
            return

        fx, fy, cx, cy = self.K
        dh, dw = self.depth.shape[:2]
        # If the depth resolution differs from the color resolution, scale the pixels
        sx = dw / self.info_size[0]
        sy = dh / self.info_size[1]

        poses = PoseArray()
        poses.header.frame_id = self.target_frame
        poses.header.stamp = msg.header.stamp
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL      # delete cylinders from the previous cycle
        markers.markers.append(clear)

        for i, det in enumerate(msg.detections):
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            # 3a) Crop the CENTRAL part of the bounding box (edges are usually floor/background)
            hw = det.bbox.size_x * self.roi / 2.0
            hh = det.bbox.size_y * self.roi / 2.0
            u0, u1 = int(max(0, (u - hw) * sx)), int(min(dw, (u + hw) * sx + 1))
            v0, v1 = int(max(0, (v - hh) * sy)), int(min(dh, (v + hh) * sy + 1))
            patch = self.depth[v0:v1, u0:u1]

            # 3b) Distance = median of valid readings (robust to noise and holes)
            valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < self.max_depth)]
            if valid.size < 10:
                self.get_logger().warn('detection without valid depth, ignoring it',
                                       throttle_duration_sec=2.0)
                continue
            z = float(np.median(valid)) + self.radius   # surface -> bottle center axis

            # 3c) Pixel + distance -> 3D point in the camera's OPTICAL frame
            #     (x to the right, y down, z forward)
            point = PointStamped()
            point.header.frame_id = msg.header.frame_id   # camera_color_optical_frame
            point.header.stamp = msg.header.stamp
            point.point.x = (u - cx) * z / fx
            point.point.y = (v - cy) * z / fy
            point.point.z = z

            # 3d) TF: from camera to target frame (odom / map / base_footprint)
            try:
                world = self.tf_buffer.transform(point, self.target_frame,
                                                 timeout=Duration(seconds=0.2))
            except Exception:
                try:   # if there's no TF for that exact timestamp, use the latest one
                    point.header.stamp = Time().to_msg()
                    world = self.tf_buffer.transform(point, self.target_frame,
                                                     timeout=Duration(seconds=0.2))
                except Exception as err:
                    self.get_logger().warn(f'TF failed: {err}', throttle_duration_sec=2.0)
                    continue

            pose = Pose()
            pose.position = world.point
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            markers.markers.append(self.make_marker(i, world, msg))

            score = det.results[0].hypothesis.score if det.results else 0.0
            self.get_logger().info(
                f'bottle {i}: x={world.point.x:.2f} y={world.point.y:.2f} '
                f'z={world.point.z:.2f} m in "{self.target_frame}" '
                f'(camera dist {z:.2f} m, conf {score:.2f})',
                throttle_duration_sec=1.0)

        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)

    # Orange cylinder in RViz where we believe each bottle is located
    def make_marker(self, idx: int, world: PointStamped, msg: Detection2DArray) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = msg.header.stamp
        m.ns = 'bottles'
        m.id = idx
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = world.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2 * self.radius
        m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.8
        m.lifetime = Duration(seconds=1.0).to_msg()
        return m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottleLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
