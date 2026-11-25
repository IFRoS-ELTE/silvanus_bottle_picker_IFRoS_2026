#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# deploy.sh - copy the image saved by build.sh to the robot and load it there
#
#   ./docker/deploy.sh                          # the last build, to agilex@192.168.1.102
#   ROBOT_IP=192.168.1.105 ./docker/deploy.sh   # the robot's address changes with DHCP
#   TAG=v1 ./docker/deploy.sh                   # a given build
#
# Asks for the robot's password (ssh, and sudo for docker load).
# ---------------------------------------------------------------------------
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-bottle_picker}"
OUT="$REPO/docker/out"
TAG="${TAG:-$(cat "$OUT/LATEST" 2>/dev/null || true)}"
ROBOT_USER="${ROBOT_USER:-agilex}"
ROBOT_IP="${ROBOT_IP:-192.168.1.102}"
ROBOT="$ROBOT_USER@$ROBOT_IP"

[ -n "$TAG" ] || { echo "No build found: run ./docker/build.sh first"; exit 1; }
TAR="$IMAGE-$TAG.tar.gz"
[ -f "$OUT/$TAR" ] || { echo "$OUT/$TAR not found: run ./docker/build.sh first"; exit 1; }

echo "Copying $TAR ($(du -h "$OUT/$TAR" | cut -f1)) to $ROBOT ..."
# rsync resumes an interrupted copy over flaky Wi-Fi
rsync -P --partial "$OUT/$TAR" "$OUT/$TAR.sha256" "$ROBOT:~/"

# -t: sudo asks for the password
ssh -t "$ROBOT" "set -e
sha256sum -c '$TAR.sha256'
gunzip -c '$TAR' | sudo docker load
sudo docker tag '$IMAGE:$TAG' '$IMAGE:latest'
sudo docker image inspect '$IMAGE:$TAG' --format 'loaded {{.RepoTags}} {{.Architecture}}'
rm -f '$TAR' '$TAR.sha256'
echo 'Old $IMAGE images (remove with: sudo docker image rm $IMAGE:<tag>):'
sudo docker images '$IMAGE'"
echo "Loaded $IMAGE:$TAG (also tagged $IMAGE:latest) on $ROBOT_IP."
