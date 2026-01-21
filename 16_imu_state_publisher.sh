#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/imu_stationary.pid
LOG_FILE=/home/dev/bags/imu_stationary.log
LAUNCH_CMD='ros2 launch owl_imu imu_stationary.launch.py'

usage(){ cat <<USAGE
Usage:
  $0 start [extra-ros-args]
  $0 stop
  $0 status
  $0 tail
Notes:
  • Runs inside container '\$NAME' (must already be running)
  • Logs to: $LOG_FILE
  • PID file: $PID_FILE
  • Launch: $LAUNCH_CMD
USAGE
}

require_running_container(){
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "❌ Container '$NAME' is not running. Start it first (e.g. ./01_start_container.sh)." >&2
    exit 1
  fi
}

start_node(){
  local extra_args=("$@")

  docker exec "$NAME" bash -lc '
    set -e

    # ROS env
    source /opt/ros/humble/setup.bash
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true

    # Ensure log dir exists
    mkdir -p "$(dirname "'"$LOG_FILE"'")"

    # Start detached; record PID; log to bags
    setsid bash -lc "'"$LAUNCH_CMD"'" '"${extra_args[@]+"${extra_args[@]}"}"' \
      > "'"$LOG_FILE"'" 2>&1 < /dev/null &

    echo $! > "'"$PID_FILE"'"
    echo "started"
  '
}

case "${1:-}" in
  start)
    shift
    require_running_container
    out=$(start_node "$@")
    [[ "$out" == "started" ]] && echo "✅ imu_stationary launch started (logging to $LOG_FILE)"
    ;;
  stop)
    require_running_container
    echo "⏹ stopping…"
    docker exec "$NAME" bash -lc '
      if [[ -f "'"$PID_FILE"'" ]]; then
        kill -INT "$(cat "'"$PID_FILE"'")" 2>/dev/null || true
        rm -f "'"$PID_FILE"'"
      else
        # fallback: kill by pattern
        pkill -INT -f "ros2 launch owl_imu imu_stationary.launch.py" 2>/dev/null || true
        pkill -INT -f "imu_stationary_publisher" 2>/dev/null || true
      fi
    '
    ;;
  status)
    require_running_container
    docker exec "$NAME" bash -lc '
      P1="ros2 launch owl_imu imu_stationary.launch.py"
      if pgrep -af "$P1" >/dev/null; then
        echo "✅ imu_stationary running:"
        pgrep -af "$P1"
      else
        echo "no imu_stationary running"
      fi
      if [[ -f "'"$LOG_FILE"'" ]]; then
        echo; echo "Log tail:"
        tail -n 20 "'"$LOG_FILE"'" || true
      fi
    '
    ;;
  tail)
    require_running_container
    docker exec -it "$NAME" bash -lc '
      target="'"$LOG_FILE"'"
      [[ -f "$target" ]] || { echo "no log file at $target"; exit 1; }
      echo "📄 Tailing $target (Ctrl+C to stop)"
      tail -n +1 -f "$target"
    '
    ;;
  *)
    usage; exit 1;;
esac
