#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# build.sh - build the robot image and save it as a tarball for the robot
#
#   ./docker/build.sh          # bottle_picker:<date-time>, saved to bottle_picker-<tag>.tar.gz
#   TAG=v1 ./docker/build.sh   # bottle_picker:v1
#
# Then copy it to the robot with ./docker/deploy.sh.
# ---------------------------------------------------------------------------
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-bottle_picker}"
TAG="${TAG:-$(date +%Y%m%d-%H%M)}"
OUT="$REPO/docker/out"

[ -f "$REPO/yolo11n.pt" ] || { echo "yolo11n.pt missing in $REPO (run the detector once on the laptop)"; exit 1; }

docker build --platform linux/amd64 -t "$IMAGE:$TAG" -t "$IMAGE:latest" "$REPO"

mkdir -p "$OUT"
TAR="$OUT/$IMAGE-$TAG.tar.gz"
echo "Saving $IMAGE:$TAG to $TAR ..."
docker save "$IMAGE:$TAG" | gzip -1 > "$TAR"
(cd "$OUT" && sha256sum "$(basename "$TAR")" > "$(basename "$TAR").sha256")
ls -lh "$TAR"
echo "$TAG" > "$OUT/LATEST"
echo "Next: ./docker/deploy.sh   (copies $IMAGE:$TAG to the robot)"
