#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"
xhost +local:root >/dev/null 2>&1 || true
docker exec -it \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e ROS_IMAGE_TRANSPORT=compressed \
  "$NAME" bash -lc '
  source /opt/ros/humble/setup.bash && rqt
'
