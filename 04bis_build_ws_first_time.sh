#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"
docker exec -it -u 0 "$NAME" bash -lc '
  cd /home/dev/ws && mkdir -p src &&
  rosdep update &&
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble &&
  source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install
'
