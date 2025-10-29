#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"
docker exec -it "$NAME" bash -lc '
  cd ~/ws && mkdir -p src &&
  if [[ -f owl_stack.repos ]]; then vcs import src < owl_stack.repos || true; else echo "⚠️ owl_stack.repos not found in ~/ws"; fi &&
  rosdep update &&
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble &&
  source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install
'
