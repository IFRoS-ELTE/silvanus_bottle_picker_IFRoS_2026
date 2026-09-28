# ---------------------------------------------------------------------------
# Student template image for the Silvanus robot.
#
#   docker build --platform linux/amd64 -t my_project:v1 .
#
# --platform linux/amd64 is REQUIRED on an Apple Silicon Mac or any other ARM
# machine. The robot computer is x86-64; an arm64 image loads without
# complaint and then dies with "exec format error".
# ---------------------------------------------------------------------------

# Same base as the robot image, so the ROS distro, RMW and message type hashes
# match. Do not switch to Humble/Kilted/Rolling: ROS 2 does not support
# cross-distro communication, and you will discover nothing.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# The robot has NO INTERNET. Anything your nodes need at run time must be
# installed here, on your laptop, at build time.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-geometry-msgs \
      python3-numpy \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src ./src
RUN source /opt/ros/jazzy/setup.bash \
 && apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && colcon build --merge-install \
 && rm -rf build log /var/lib/apt/lists/*

# The robot's ROS domain. student-run.sh sets this too; belt and braces.
ENV ROS_DOMAIN_ID=42

COPY entrypoint.sh /usr/local/bin/entrypoint.sh

# Do not run as root on shared hardware.
RUN useradd -m -u 1001 student && chown -R student:student /ws
USER student

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
