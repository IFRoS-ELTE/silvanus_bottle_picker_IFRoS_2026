import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import zmq

SERVER_TIMEOUT_MS = 30000

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
        self.latest_bbox = [150, 200, 350, 450] # Placeholder YOLO bounding box
        self.is_predicting = False

    def connect_to_server(self):
        # connect() never fails even if the server is down, so replies are polled with a timeout
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.setsockopt(zmq.LINGER, 0)
        self.zmq_socket.connect("tcp://localhost:5555")

    def pointcloud_callback(self, msg: PointCloud2):
        # 1. Print this exactly ONCE so we know the topic connection is working
        self.get_logger().info('Point cloud received from simulation!', once=True)

        if self.is_predicting:
            return # Don't pile up requests while waiting for the GPU
            
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
        
        # 3. Use a smaller dummy bounding box just in case the sim camera is low resolution
        xmin, ymin, xmax, ymax = 50, 50, 150, 150 
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