"""BOttle detection with YOLO.

Takes each image from the camera, passes it to YOLO, keeps
only the "bottle" class detections, and publishes (1) the list of detections and
(2) the image with bounding boxes drawn to view in rqt_image_view / RViz.

    Input   : /camera/color/image_raw            sensor_msgs/Image
    Outputs : /bottles/detections                vision_msgs/Detection2DArray
              /bottles/image_annotated           sensor_msgs/Image

Parameters (change with --ros-args -p name:=value):
    model         YOLO weights. 'yolo11n.pt' = small model pre-trained on COCO,
                  which already knows the "bottle" class. Later on: your own custom weights.
    image_topic   Camera topic.
    confidence    Minimum confidence threshold (0-1). Lower = detects more, but with more false positives.
    classes       List of class names to keep. Default is only 'bottle'.
    device        '' = automatic (GPU if available), 'cpu', or '0' for the first GPU.
"""

import os
import signal
import sys
import time

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from cv_bridge import CvBridge

    from ultralytics import YOLO
except ImportError as err:
    # This needs a Python with both ROS and ultralytics: the venv from the README
    # (python3 -m venv --system-site-packages). Started under another one (e.g. conda's
    # (base), which can't load ROS, or the system one without ultralytics), restart under it.
    venv = os.path.expanduser(os.environ.get('SILVANUS_VENV', '~/silvanus_venv'))
    venv_python = os.path.join(venv, 'bin', 'python3')
    if os.path.exists(venv_python) and os.path.realpath(sys.prefix) != os.path.realpath(venv):
        os.execv(venv_python, [venv_python] + sys.argv)
    sys.exit(f'bottle_detector: {err}. It needs ROS and ultralytics in the same Python: run '
             f'"conda deactivate" and "source {venv}/bin/activate" (venv set up as in the '
             'README), or set SILVANUS_VENV to your venv.')


class BottleDetector(Node):
    def __init__(self) -> None:
        super().__init__('bottle_detector')

        # --- Parameters ---------------------------------------------------
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('classes', ['bottle'])
        self.declare_parameter('device', '')

        model_path = self.get_parameter('model').value
        image_topic = self.get_parameter('image_topic').value
        self.confidence = float(self.get_parameter('confidence').value)
        wanted = list(self.get_parameter('classes').value)
        self.device = self.get_parameter('device').value or None

        # --- Upload YOLO --------------------------------------------------
        # La primera vez descarga los pesos (~5 MB) a la carpeta desde la que lanzas.
        self.get_logger().info(f'cargando YOLO: {model_path} ...')
        self.model = YOLO(model_path)
        # model.names diccionary {id: nombre}, p. ej. {39: 'bottle', ...}
        self.class_ids = [i for i, name in self.model.names.items() if name in wanted]
        if not self.class_ids:
            self.get_logger().error(
                f'ninguna de las clases {wanted} existe en el modelo. '
                f'Clases disponibles: {list(self.model.names.values())}')
            raise SystemExit(1)
        self.get_logger().info(
            f'buscando clases {wanted} (ids {self.class_ids}), confianza >= {self.confidence}')

        # --- ROS: inputsvand outputs ---------------------------------------
        self.bridge = CvBridge()  # convierte sensor_msgs/Image <-> imagen de OpenCV
        # Profundidad 1 + BEST_EFFORT: si YOLO va mas lento que la camara, nos
        # quedamos siempre con la imagen MAS RECIENTE y se descartan las viejas.
        sensor_qos = QoSProfile(depth=1,
                                history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.det_pub = self.create_publisher(Detection2DArray, '/bottles/detections', 10)
        self.img_pub = self.create_publisher(Image, '/bottles/image_annotated', 10)

        self._frames = 0
        self._t0 = time.monotonic()
        self.get_logger().info(f'esperando imagenes en {image_topic} ...')

    def on_image(self, msg: Image) -> None:
        # 1) ROS Image -> matrix of OpenCV (BGR,what  YOLO expects)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2) Inferencia. [0] one only image
        result = self.model.predict(frame,
                                    conf=self.confidence,
                                    classes=self.class_ids,
                                    device=self.device,
                                    verbose=False)[0]

        # 3) COnvert the YOLO detection with a massagr standar ROS
        out = Detection2DArray()
        out.header = msg.header  # mismo instante y frame que la imagen original
        for xywh, conf, cls in zip(result.boxes.xywh.tolist(),
                                   result.boxes.conf.tolist(),
                                   result.boxes.cls.tolist()):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = xywh[0]   # rectangle center x  (pixeles)
            det.bbox.center.position.y = xywh[1]   # rectangle center y (pixeles)
            det.bbox.size_x = xywh[2]              # widht (pixels)
            det.bbox.size_y = xywh[3]              # high  (pixels)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[int(cls)]
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)

        # 4) (Imagen con los recuadros dibujados, para tenerlo visualmente)
        annotated = result.plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header = msg.header
        self.img_pub.publish(img_msg)

        # 5) Un log each second: FPS y cuantas botellas vemos
        self._frames += 1
        elapsed = time.monotonic() - self._t0
        if elapsed >= 2.0:
            fps = self._frames / elapsed
            self._frames, self._t0 = 0, time.monotonic()
            scores = ', '.join(f'{d.results[0].hypothesis.score:.2f}' for d in out.detections)
            self.get_logger().info(
                f'{fps:.1f} FPS | botellas: {len(out.detections)} [{scores}]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C under ros2 launch delivers SIGINT twice (terminal + launch); don't let the
        # second one interrupt the cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
