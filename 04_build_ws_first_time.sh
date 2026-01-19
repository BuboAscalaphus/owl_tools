#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"

docker exec -it "$NAME" bash -lc '
  set -eo pipefail

  set +u
  source /opt/ros/"$ROS_DISTRO"/setup.bash
  set -u

  cd /home/dev/ws
  colcon build --symlink-install
  
'

