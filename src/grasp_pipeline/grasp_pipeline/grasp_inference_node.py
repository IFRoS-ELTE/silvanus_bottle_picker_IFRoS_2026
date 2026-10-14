import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import zmq

SERVER_TIMEOUT_MS = 30000
# A detection older than this relative to the point cloud is not used for cropping
DETECTION_MAX_AGE_S = 1.0

class GraspInferenceClient(Node):
    def __init__(self):
        super().__init__('grasp_inference_client')
        
        # 1. Setup ZMQ Client to talk to the Python 3.7 Conda environment
        self.get_logger().info('Connecting to Contact-GraspNet server...')
        self.zmq_context = zmq.Context()
        self.connect_to_server()

        # 2. Subscribe to RealSense (Updated for simulation topic)
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

    def get_bottle_bbox(self, cloud_msg: PointCloud2):
        """Return the best bottle box as (xmin, ymin, xmax, ymax) in cloud pixels, or None."""
        if self.latest_detections is None or self.color_size is None:
            self.get_logger().warn('Waiting for /bottles/detections and /camera/color/camera_info...',
                                   throttle_duration_sec=2.0)
            return None

        age = Time.from_msg(cloud_msg.header.stamp) - Time.from_msg(self.latest_detections.header.stamp)
        if abs(age.nanoseconds) > DETECTION_MAX_AGE_S * 1e9:
            self.get_logger().warn('Latest bottle detection is too old for this point cloud.',
                                   throttle_duration_sec=2.0)
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
        
        # 3. Crop the cloud to the detected bottle
        xmin, ymin, xmax, ymax = bbox
        cropped_3d_region = points_3d[ymin:ymax, xmin:xmax, :]
        
        flattened_crop = cropped_3d_region.reshape(-1, 3)
        clean_points = flattened_crop[~np.isnan(flattened_crop).any(axis=1)].astype(np.float32)

        if clean_points.shape[0] < 100:
            self.get_logger().warn('Not enough valid depth points in bounding box.')
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
        reply_bytes = self.zmq_socket.recv()
        reply_array = np.frombuffer(reply_bytes, dtype=np.float32)
        
        if reply_array.size == 16:
            best_4x4_matrix = reply_array.reshape(4, 4)
            self.get_logger().info(f'Received Grasp Pose:\n{best_4x4_matrix}')
            # TODO: Publish to tf2
        else:
            self.get_logger().warn('No viable grasps returned from server.')
            
        self.is_predicting = False

def main(args=None):
    rclpy.init(args=args)
    node = GraspInferenceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()