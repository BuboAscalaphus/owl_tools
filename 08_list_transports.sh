#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"
docker exec -it "$NAME" bash -lc '
  source /opt/ros/humble/setup.bash &&
  ros2 run image_transport list_transports
'
