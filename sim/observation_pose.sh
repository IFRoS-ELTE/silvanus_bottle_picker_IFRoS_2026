#!/usr/bin/env bash
#Takes the arm to the position of observation camera , first
ros2 run scout_xarm_teleop xarm6_init_home --duration 4.0 \
  --positions 0.0 1.5775 -1.4176 0.0267 -1.1761 0.0
