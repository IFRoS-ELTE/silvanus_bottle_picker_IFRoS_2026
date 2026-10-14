#!/usr/bin/env bash
# Postura "grasp": cámara cerca, apuntando a la botella de pie de la izquierda.
# Ángulos joint1..joint6 (rad), medidos en simulación el 14-10-2026.
#
#   ./sim/grasp_pose.sh          -> simulación (Gazebo, dominio 0)
#   ./sim/grasp_pose.sh --real   -> robot real (dentro del contenedor silvanus, dominio 42)
set -euo pipefail

POSITIONS="0.2939 1.2808 -1.1217 -0.1209 -1.2198 0.0"

if [ "${1:-}" = "--real" ]; then
    echo "!! ROBOT REAL: el brazo se va a mover. Persona en la seta de emergencia. Ctrl-C para cancelar (5 s)..."
    sleep 5
    # Más lento en el real (10 s). Se ejecuta dentro del contenedor del hardware, que ya tiene scout_xarm_teleop.
    sudo docker exec -it -e ROS_DOMAIN_ID=42 silvanus bash -c \
      "source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash && \
       ros2 run scout_xarm_teleop xarm6_init_home --duration 10.0 --positions $POSITIONS"
else
    ros2 run scout_xarm_teleop xarm6_init_home --duration 4.0 --positions $POSITIONS
fi
