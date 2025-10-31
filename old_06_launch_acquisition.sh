#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"
docker exec -it "$NAME" bash -lc '
  source /opt/ros/humble/setup.bash &&
  source ~/ws/install/setup.bash &&
  ros2 launch owl_sense acquisition.launch.py
'
