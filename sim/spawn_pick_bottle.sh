#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# spawn_pick_bottle.sh - one bottle inside the arm's reach, for testing pick_bottle
#
#   ./sim/spawn_pick_bottle.sh                          # standing bottle at x=0.6 y=0
#   ./sim/spawn_pick_bottle.sh weston_robot_empty 0.62 0.1 lying
#
# The xArm6 can only reach the ground ~0.5-0.65 m in front of base_footprint
# (the chassis is in the way closer, the arm is too short further out).
# ---------------------------------------------------------------------------
set -euo pipefail

WORLD="${1:-weston_robot_empty}"
X="${2:-0.6}"
Y="${3:-0.0}"
POSE="${4:-standing}"
MODEL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/models/bottle/model.sdf"

if [ "$POSE" = lying ]; then Z=0.035; ROLL=1.5708; else Z=0.0; ROLL=0; fi
ros2 run ros_gz_sim create -world "$WORLD" -file "$MODEL" -name bottle_pick \
    -x "$X" -y "$Y" -z "$Z" -R "$ROLL" -P 0 -Y 0
