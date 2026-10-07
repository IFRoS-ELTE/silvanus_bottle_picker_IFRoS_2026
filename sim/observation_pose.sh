#!/usr/bin/env bash
# Lleva el brazo a la postura de observación (cámara mirando al suelo delante del robot).
# Ángulos joint1..joint6 en radianes, medidos en simulación el 05-10-2026.
ros2 run scout_xarm_teleop xarm6_init_home --duration 4.0 \
  --positions 0.0 0.6485 -1.9267 0.3359 0.6134 -0.9983
