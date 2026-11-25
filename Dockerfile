# ---------------------------------------------------------------------------
# Bottle picker image for the Silvanus robot: YOLO detector, bottle localizer,
# grasp_inference and pick_bottle, run beside the robot's own silvanus container.
#
#   ./docker/build.sh             # build, then save to bottle_picker-<tag>.tar.gz
#   ./docker/deploy.sh            # copy the tarball to the robot and docker load it
#
# See "Running on the robot (Docker)" in README.md and silvanus_robot/STUDENT_GUIDE.md.
#
# --platform linux/amd64 is REQUIRED on an ARM machine: the robot computer is x86-64.
# ---------------------------------------------------------------------------

# Same base (by digest) as the robot image, so the ROS distro, RMW and message type
# hashes match. ROS 2 does not talk across distros.
FROM ros:jazzy-ros-base@sha256:bab7e640bf79cd84957e4e18fcba7d87efc3385b4e3f36a32eeca01638e43206

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# The robot has NO INTERNET: everything the nodes need at run time is installed here.
# Listed by hand instead of rosdep: grasp_pipeline's package.xml also names the sim
# packages (scout_xarm_gazebo, xarm_moveit_config, ...), which are not apt packages
# and are not needed on the robot.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-cv-bridge \
      ros-jazzy-vision-msgs \
      ros-jazzy-sensor-msgs-py \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-control-msgs \
      ros-jazzy-moveit-msgs \
      ros-jazzy-visualization-msgs \
      python3-numpy \
      python3-zmq \
      python3-pip \
      libgl1 \
      libglib2.0-0t64 \
 && rm -rf /var/lib/apt/lists/*

# YOLO. CPU-only torch: student-run.sh gives the container no GPU, and the CUDA wheels
# would add several GB to an image that travels over Wi-Fi. numpy<2: ROS 2 Jazzy's
# cv_bridge is built against numpy 1.x. Versions as on the laptop that was tested.
RUN pip install --no-cache-dir --break-system-packages \
      --index-url https://download.pytorch.org/whl/cpu \
      torch==2.14.1 torchvision==0.29.1 \
 && pip install --no-cache-dir --break-system-packages \
      ultralytics==8.4.175 "numpy<2"

WORKDIR /ws
COPY src/bottle_perception src/bottle_perception
COPY src/grasp_pipeline src/grasp_pipeline
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --merge-install \
 && rm -rf build log

# The detector's default model 'yolo11n.pt' is relative to the working directory (/ws)
COPY yolo11n.pt /ws/yolo11n.pt

# The robot's ROS domain. student-run.sh sets this too.
ENV ROS_DOMAIN_ID=42
# Ultralytics must not try the network (no internet on the robot: checks would stall)
ENV YOLO_OFFLINE=true
ENV YOLO_CONFIG_DIR=/home/student/.config/Ultralytics
# DDS over UDP only, no shared memory: the silvanus container runs as root and this one as
# 'student' with no capabilities, so neither can open the other's Fast DDS shared-memory
# files, and with them on, the two never see each other's nodes. (Tested with a root
# container on the same host: no nodes discovered with shared memory, all with UDPv4.)
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/pick-bottle /usr/local/bin/pick-bottle

# Do not run as root on shared hardware.
RUN useradd -m -u 1001 student \
 && mkdir -p "$YOLO_CONFIG_DIR" \
 && chown -R student:student /ws /home/student
USER student

# Load the model and run it once now, as the user that runs it on the robot, so nothing
# is missing at run time (Ultralytics writes its settings under ~/.config on first use)
RUN source /ws/install/setup.bash && python3 -c "import numpy as np; from ultralytics import YOLO; \
r = YOLO('yolo11n.pt').predict(np.zeros((480, 640, 3), np.uint8), device='cpu', verbose=False)[0]; r.plot(); \
import cv_bridge, torch; print('YOLO OK, torch', torch.__version__)"

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
