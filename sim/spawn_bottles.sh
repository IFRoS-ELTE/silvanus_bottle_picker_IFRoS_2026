#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# spawn_bottles.sh - add bottles in the gazebo which is already running
#
#   ./sim/spawn_bottles.sh                         # mundo por defecto: weston_robot_empty
#   ./sim/spawn_bottles.sh silvanus_block_arena    # otro mundo (nombre SIN .world)
#
# Requisitos: terminal con ROS 2 Jazzy + scout_xarm_ros2 sourceados y ROS_DOMAIN_ID=0.
# El robot aparece en (0,0) mirando hacia +x, asi que las botellas van delante.
# ---------------------------------------------------------------------------
set -euo pipefail

WORLD="${1:-weston_robot_empty}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL="$HERE/models/bottle/model.sdf"

[ -f "$MODEL" ] || { echo "No encuentro $MODEL"; exit 1; }

spawn() {  # nombre x y z roll pitch yaw
    echo "==> $1 en x=$2 y=$3 (roll=$5)"
    ros2 run ros_gz_sim create -world "$WORLD" -file "$MODEL" -name "$1" \
        -x "$2" -y "$3" -z "$4" -R "$5" -P "$6" -Y "$7"
}

#      nombre           x     y     z      roll    pitch yaw
spawn  bottle_front     1.0   0.0   0.0    0       0     0      # de pie, justo delante
spawn  bottle_left      1.5   0.6   0.0    0       0     0      # de pie, a la izquierda
spawn  bottle_lying     1.3  -0.5   0.035  1.5708  0     0.5    # tumbada, a la derecha

echo "Done. To remove: gz service -s /world/$WORLD/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: \"bottle_front\" type: MODEL'"
