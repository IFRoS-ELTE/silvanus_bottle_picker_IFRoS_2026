#!/usr/bin/env bash
# Sources ROS 2 and your workspace, then runs whatever command it was given.
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
exec "$@"
