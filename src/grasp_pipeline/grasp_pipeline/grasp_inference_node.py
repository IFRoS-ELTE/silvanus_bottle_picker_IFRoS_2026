import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
import zmq

from grasp_pipeline.geometry import matrix_to_pose, matrix_to_quaternion, transform_to_matrix

SERVER_TIMEOUT_MS = 30000
# Server reply: per grasp a row-major 4x4 matrix then its score, best first
FLOATS_PER_GRASP = 17

class GraspInferenceClient(Node):
    def __init__(self):
        super().__init__('grasp_inference_client')

        # Gazebo's rgbd point cloud uses camera-link axes (x forward, y left, z up) even though
        # the bridge labels it camera_depth_optical_frame. A real RealSense publishes true
        # optical axes (x right, y down, z forward): use cloud_axes:=optical there.
        self.declare_parameter('cloud_axes', 'gazebo')
        self.cloud_axes = self.get_parameter('cloud_axes').value
        # Keep cloud points this close to the bottle centre (m). 0.15 already lets the robot's
        # front bumper in from the side view, and grasps drift 2-5 cm off the bottle.
        self.declare_parameter('crop_radius', 0.10)
        self.crop_radius = float(self.get_parameter('crop_radius').value)
        # Re-centre the grasp on the points between the fingers (see refine_grasp)
        self.declare_parameter('refine_grasp', True)
        self.declare_parameter('gripper_half_opening', 0.0425)  # xArm gripper opens 85 mm
        self.refine = bool(self.get_parameter('refine_grasp').value)
        self.half_opening = float(self.get_parameter('gripper_half_opening').value)
        # false: only estimate and publish the bottle, never call the Contact-GraspNet server
        self.declare_parameter('use_graspnet', True)
        self.use_graspnet = bool(self.get_parameter('use_graspnet').value)
        # Point cloud source. Empty depth_topic: the organized cloud on /camera/depth/points
        # (Gazebo). Set (e.g. the real D435's /camera/camera/aligned_depth_to_color/image_raw):
        # the cloud is built here from that depth image and its camera_info, which must be
        # aligned with the color image the YOLO boxes come from. Then cloud_axes is ignored
        # (the points are built in optical axes).
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('depth_info_topic', '')  # default: camera_info next to depth_topic
        self.declare_parameter('max_rate', 5.0)         # clouds processed per second at most
        # true over Wi-Fi (real robot): best effort drops a whole image if a fragment is lost
        self.declare_parameter('reliable', False)
        # A detection older than this relative to the cloud is not used for cropping (s).
        # Raise it when detections come slowly; fine while the camera and bottle are still.
        self.declare_parameter('detection_max_age', 1.0)
        self.detection_max_age = float(self.get_parameter('detection_max_age').value)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_info_topic = (self.get_parameter('depth_info_topic').value
                                 or self.depth_topic.rsplit('/', 1)[0] + '/camera_info')
        self.min_period = 1.0 / max(float(self.get_parameter('max_rate').value), 1e-3)
        self.last_processed = 0.0
        self.depth_K = None
        # Frame whose z = 0 is the ground; the bottle axis is estimated and published in it
        self.declare_parameter('ground_frame', 'base_footprint')
        self.declare_parameter('ground_clearance', 0.01)  # points lower than this are ground
        self.ground_frame = self.get_parameter('ground_frame').value
        self.ground_clearance = float(self.get_parameter('ground_clearance').value)
        # Points reaching higher than this: standing bottle (vertical axis), else lying
        self.declare_parameter('lying_max_height', 0.12)
        self.lying_max_height = float(self.get_parameter('lying_max_height').value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        if self.cloud_axes not in ('gazebo', 'optical'):
            raise ValueError("cloud_axes must be 'gazebo' or 'optical'")
        
        # 1. Setup ZMQ Client to talk to the Python 3.7 Conda environment
        self.zmq_context = zmq.Context()
        if self.use_graspnet:
            self.get_logger().info('Connecting to Contact-GraspNet server...')
            self.connect_to_server()
        else:
            self.get_logger().info('use_graspnet is false: only estimating the bottle')

        # 2. Point cloud: Gazebo's organized cloud, or built here from a depth image
        if self.depth_topic:
            image_qos = qos_profile_sensor_data
            if self.get_parameter('reliable').value:
                image_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                       reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Image, self.depth_topic, self.depth_callback, image_qos)
            self.create_subscription(CameraInfo, self.depth_info_topic, self.depth_info_callback,
                                     qos_profile_sensor_data)
            self.get_logger().info(f'Building the point cloud from {self.depth_topic} '
                                   f'and {self.depth_info_topic}')
        else:
            self.subscription = self.create_subscription(
                PointCloud2,
                '/camera/depth/points',
                self.pointcloud_callback,
                10
            )

        # 3. Subscribe to the YOLO bounding boxes from bottle_perception
        self.create_subscription(
            Detection2DArray,
            '/bottles/detections',
            self.detections_callback,
            10
        )
        # The boxes are in color image pixels, so the color resolution is needed to map them onto the cloud
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        # Grasps in the point cloud's frame, Contact-GraspNet convention
        # (z = approach, x = finger closing direction, fingers 0.1034 m along z).
        # /grasp/pose is the best-scoring one, /grasp/candidates all of them, best score first.
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp/pose', 10)
        self.candidates_pub = self.create_publisher(PoseArray, '/grasp/candidates', 10)
        # Bottle estimated from the same cloud, as a cylinder in ground_frame with the cloud's
        # stamp: pose = centre, orientation z = axis, scale.x = diameter, scale.z = length.
        # Also viewable in RViz.
        self.bottle_pub = self.create_publisher(Marker, '/grasp/bottle', 10)
        self.latest_detections = None
        self.color_size = None # (width, height) of the color image
        self.is_predicting = False

    def connect_to_server(self):
        # connect() never fails even if the server is down, so replies are polled with a timeout
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.setsockopt(zmq.LINGER, 0)
        self.zmq_socket.connect("tcp://localhost:5555")

    def detections_callback(self, msg: Detection2DArray):
        self.latest_detections = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.color_size = (msg.width, msg.height)

    def depth_info_callback(self, msg: CameraInfo):
        self.depth_K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def depth_to_points(self, msg: Image):
        """Organized (H, W, 3) cloud in the depth image's optical frame, NaN where no depth."""
        if msg.encoding in ('16UC1', 'mono16'):    # real RealSense: millimetres
            dtype, scale = np.uint16, 0.001
        elif msg.encoding == '32FC1':               # Gazebo: metres
            dtype, scale = np.float32, 1.0
        else:
            self.get_logger().error(f'Unsupported depth encoding {msg.encoding}', once=True)
            return None
        dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
        row = msg.step // dt.itemsize
        depth = (np.frombuffer(msg.data, dtype=dt).reshape(msg.height, row)[:, :msg.width]
                 .astype(np.float32) * scale)
        depth[~(depth > 0)] = np.nan              # 0 = no reading
        fx, fy, cx, cy = self.depth_K
        v, u = np.indices(depth.shape, dtype=np.float32)
        return np.dstack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth])

    def depth_callback(self, msg: Image):
        self.get_logger().info('Depth image received', once=True)
        if self.is_predicting:
            return
        if self.depth_K is None:
            self.get_logger().warn(f'Waiting for {self.depth_info_topic}...',
                                   throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        if now - self.last_processed < self.min_period:
            return
        bbox = self.get_bottle_bbox(msg)  # an Image has the same header/width/height
        if bbox is None:
            return
        self.last_processed = now
        self.is_predicting = True
        points_3d = self.depth_to_points(msg)
        if points_3d is None:
            self.is_predicting = False
            return
        try:
            self.process_points(msg, points_3d, gazebo_axes=False)
        finally:
            self.is_predicting = False  # also if something raised

    def get_bottle_bbox(self, cloud_msg: PointCloud2):
        """Return the best bottle box as (xmin, ymin, xmax, ymax) in cloud pixels, or None."""
        if self.latest_detections is None or self.color_size is None:
            self.get_logger().warn('Waiting for /bottles/detections and /camera/color/camera_info...',
                                   throttle_duration_sec=2.0)
            return None

        age = Time.from_msg(cloud_msg.header.stamp) - Time.from_msg(self.latest_detections.header.stamp)
        if abs(age.nanoseconds) > self.detection_max_age * 1e9:
            self.get_logger().warn(
                f'Latest bottle detection is {abs(age.nanoseconds) / 1e9:.1f} s older than this '
                f'point cloud (detection_max_age = {self.detection_max_age} s). Is the detector '
                'getting images? Over Wi-Fi use reliable:=true.', throttle_duration_sec=2.0)
            return None

        detections = [d for d in self.latest_detections.detections if d.results]
        if not detections:
            self.get_logger().warn('No bottle detected.', throttle_duration_sec=2.0)
            return None
        best = max(detections, key=lambda d: d.results[0].hypothesis.score)

        # Scale from color pixels to cloud pixels in case the two resolutions differ
        sx = cloud_msg.width / self.color_size[0]
        sy = cloud_msg.height / self.color_size[1]
        u, v = best.bbox.center.position.x, best.bbox.center.position.y
        hw, hh = best.bbox.size_x / 2.0, best.bbox.size_y / 2.0
        xmin = int(max(0, (u - hw) * sx))
        xmax = int(min(cloud_msg.width, (u + hw) * sx + 1))
        ymin = int(max(0, (v - hh) * sy))
        ymax = int(min(cloud_msg.height, (v + hh) * sy + 1))
        return xmin, ymin, xmax, ymax

    def crop_bottle(self, points_3d, bbox):
        """Points of the box that lie within crop_radius of the bottle, or None.

        The 2D box also covers whatever is in front of or behind the bottle (the robot's own
        gripper and bumper), and Contact-GraspNet happily grasps those. The bottle centre is
        taken as the median of the middle of the box, like bottle_localizer does.
        """
        xmin, ymin, xmax, ymax = bbox
        box = points_3d[ymin:ymax, xmin:xmax, :]
        h, w = box.shape[:2]
        mh, mw = max(1, int(h * 0.15)), max(1, int(w * 0.15))
        middle = box[h // 2 - mh:h // 2 + mh + 1, w // 2 - mw:w // 2 + mw + 1].reshape(-1, 3)
        # Gazebo marks pixels beyond the depth range as +-inf, a real camera as NaN
        middle = middle[np.isfinite(middle).all(axis=1)]
        if middle.shape[0] < 10:
            return None
        center = np.median(middle, axis=0)

        points = box.reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        near = np.linalg.norm(points - center, axis=1) < self.crop_radius
        return points[near].astype(np.float32)

    def refine_grasp(self, G, points, half_side=0.04, finger_depth=0.1034, below=0.03):
        """Shift a Contact-GraspNet grasp so the fingers are centred on what is between them.

        On a thin, half-visible bottle neck the network's grasp centre lands 2-4 cm off,
        enough for the fingers to close on air. Take the cloud points in the closing region
        (grasp frame: |x| < half opening, |y| < half_side, just above the finger midpoint
        along z) and move the grasp to the middle of their x and y extent.
        """
        G = G.astype(np.float64)
        pts = points.astype(np.float64)
        for _ in range(3):
            fingers = G[:3, 3] + finger_depth * G[:3, 2]
            local = (pts - fingers) @ G[:3, :3]
            inside = ((np.abs(local[:, 0]) < self.half_opening) & (np.abs(local[:, 1]) < half_side)
                      & (local[:, 2] > -below) & (local[:, 2] < 0.0))
            if inside.sum() < 20:
                break
            dx = (local[inside, 0].min() + local[inside, 0].max()) / 2.0
            dy = (local[inside, 1].min() + local[inside, 1].max()) / 2.0
            G[:3, 3] += G[:3, 0] * dx + G[:3, 1] * dy
        return G

    @staticmethod
    def estimate_bottle(points, camera, ground_clearance=0.01, lying_max_height=0.12):
        """(centre, unit axis, radius, length) of the bottle as a cylinder, or None.

        `points` and `camera` (the camera position) are in a frame whose z = 0 is the ground.
        A bottle on the ground is either standing or lying, so the axis is taken as vertical
        if the points reach higher than `lying_max_height`, else as horizontal with its
        direction from the points' spread in xy. A free 3D fit (PCA) is avoided because depth
        noise spreads the points along the camera rays, which a noisy sensor makes the
        biggest spread. Only the camera-facing half of the bottle is visible, and those points
        sit about pi/4 * radius in front of the axis, so the axis is moved back by that much.
        Height and length only cover the cropped points (crop_radius), so a long bottle comes
        out shorter than it is.
        """
        pts = points[points[:, 2] > ground_clearance].astype(np.float64)
        if len(pts) < 50:
            return None
        mean = pts.mean(axis=0)
        top = np.percentile(pts[:, 2], 98)
        standing = top > lying_max_height
        if standing:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            xy = pts[:, :2] - mean[:2]
            direction = np.linalg.svd(xy, full_matrices=False)[2][0]
            axis = np.array([direction[0], direction[1], 0.0])
        view = mean - camera
        away = view - (view @ axis) * axis
        if np.linalg.norm(away) < 1e-6:  # looking straight down the axis: no width to measure
            return None
        away /= np.linalg.norm(away)
        across = np.cross(axis, away)
        # The camera samples the bottle evenly across its width, so the middle 80% of the points
        # span 0.8 of the diameter; the outer 10% each side are where noisy stray points are
        width = np.percentile((pts - mean) @ across, [10, 90])
        radius = (width[1] - width[0]) / (2.0 * 0.8)
        centre = mean + away * (np.pi / 4.0) * radius
        if standing:  # from the ground up to the top
            centre[2] = top / 2.0
            length = top
        else:
            along = np.percentile((pts - centre) @ axis, [2, 98])
            centre = centre + axis * (along[0] + along[1]) / 2.0
            length = along[1] - along[0]
        return centre, axis, radius, length

    def publish_bottle(self, msg, points):
        """Estimate the bottle from the crop (cloud frame) and publish it in ground_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(self.ground_frame, msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException:
            try:  # nothing for that exact stamp yet; the camera is still when it matters
                tf = self.tf_buffer.lookup_transform(self.ground_frame, msg.header.frame_id,
                                                     Time())
            except tf2_ros.TransformException as err:
                self.get_logger().warn(f'No bottle estimate, TF failed: {err}',
                                       throttle_duration_sec=2.0)
                return
        T = transform_to_matrix(tf.transform)
        world = points.astype(np.float64) @ T[:3, :3].T + T[:3, 3]
        bottle = self.estimate_bottle(world, T[:3, 3], self.ground_clearance,
                                      self.lying_max_height)
        if bottle is None:
            self.get_logger().warn('No bottle estimate: too few points above the ground.',
                                   throttle_duration_sec=2.0)
            return
        centre, axis, radius, length = bottle
        # Any rotation whose z axis is the bottle axis
        x = np.cross([0.0, 1.0, 0.0] if abs(axis[1]) < 0.9 else [1.0, 0.0, 0.0], axis)
        x /= np.linalg.norm(x)
        R = np.column_stack([x, np.cross(axis, x), axis])
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = self.ground_frame
        marker.ns = 'bottle'
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in centre)
        o = marker.pose.orientation
        o.x, o.y, o.z, o.w = (float(v) for v in matrix_to_quaternion(R))
        marker.scale.x = marker.scale.y = float(2.0 * radius)
        marker.scale.z = float(length)
        marker.color.g, marker.color.b, marker.color.a = 0.8, 0.3, 0.6
        marker.lifetime.sec = 2
        self.bottle_pub.publish(marker)

    def pointcloud_callback(self, msg: PointCloud2):
        # 1. Print this exactly ONCE so we know the topic connection is working
        self.get_logger().info('Point cloud received from simulation!', once=True)

        if self.is_predicting:
            return # Don't pile up requests while waiting for the GPU

        bbox = self.get_bottle_bbox(msg)
        if bbox is None:
            return

        self.is_predicting = True
        
        # read_points() returns a structured array on Jazzy, which cannot be reshaped to (H, W, 3)
        cloud = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)

        # 2. Safety catch: Simulations sometimes output "unorganized" clouds
        try:
            points_3d = cloud.reshape((msg.height, msg.width, 3))
        except ValueError:
            self.get_logger().error(f'Simulation point cloud is unorganized (Height: {msg.height}, Width: {msg.width}). Cannot crop with 2D bounding box.', once=True)
            self.is_predicting = False
            return
        
        try:
            self.process_points(msg, points_3d, self.cloud_axes == 'gazebo', bbox)
        finally:
            self.is_predicting = False  # also if something raised

    def process_points(self, msg, points_3d, gazebo_axes, bbox=None):
        """Crop the organized cloud to the bottle, estimate it, and optionally get grasps.

        `msg` is the cloud or depth image the points come from (header = frame and stamp).
        """
        if bbox is None:
            bbox = self.get_bottle_bbox(msg)
            if bbox is None:
                self.is_predicting = False
                return
        # 3. Crop the cloud to the detected bottle
        clean_points = self.crop_bottle(points_3d, bbox)
        if clean_points is None:
            self.get_logger().warn('No valid depth at the centre of the bottle box.')
            self.is_predicting = False
            return
        if gazebo_axes:
            # Contact-GraspNet expects optical axes, and the grasp is published in the optical frame
            clean_points = np.stack([-clean_points[:, 1], -clean_points[:, 2], clean_points[:, 0]],
                                    axis=1)

        if clean_points.shape[0] < 100:
            self.get_logger().warn('Not enough valid depth points in bounding box.')
            self.is_predicting = False
            return

        self.publish_bottle(msg, clean_points)
        if not self.use_graspnet:
            self.is_predicting = False
            return

        # Send raw bytes to the Conda server
        self.zmq_socket.send(clean_points.tobytes())
        
        # Wait for the 4x4 matrix reply
        if not self.zmq_socket.poll(SERVER_TIMEOUT_MS):
            self.get_logger().error(
                f'No reply from Contact-GraspNet server within {SERVER_TIMEOUT_MS / 1000:.0f} s. '
                'Is grasp_server.py running, and is it stuck on "Predicting..."?')
            # A REQ socket cannot send again until it gets a reply, so it has to be recreated
            self.zmq_socket.close()
            self.connect_to_server()
            self.is_predicting = False
            return
        reply_array = np.frombuffer(self.zmq_socket.recv(), dtype=np.float32)
        self.is_predicting = False
        if reply_array.size == 16:  # older grasp_server.py: best grasp only
            grasps = [reply_array.reshape(4, 4)]
        elif reply_array.size and reply_array.size % FLOATS_PER_GRASP == 0:
            rows = reply_array.reshape(-1, FLOATS_PER_GRASP)
            grasps = [row[:16].reshape(4, 4) for row in rows]
        else:
            self.get_logger().warn('No viable grasps returned from server.')
            return
        if self.refine:
            grasps = [self.refine_grasp(G, clean_points) for G in grasps]
        grasps = [G.astype(np.float64) for G in grasps]
        self.get_logger().info(f'Received {len(grasps)} grasps, best:\n{grasps[0]}')

        candidates = PoseArray()
        candidates.header = msg.header
        candidates.poses = [matrix_to_pose(G) for G in grasps]
        self.candidates_pub.publish(candidates)
        best = PoseStamped()
        best.header = msg.header
        best.pose = candidates.poses[0]
        self.grasp_pub.publish(best)


def main(args=None):
    rclpy.init(args=args)
    node = GraspInferenceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()