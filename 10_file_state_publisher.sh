#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/file_state_pub.pid
STATUS_FILE=/home/dev/bags/file_status.txt
DEFAULT_PYFILE=/home/dev/ws/src/owl_sense/owl_sense/file_state_publisher.py

usage(){ cat <<USAGE
Usage:
  $0 start [extra-ros-args]
  $0 stop
  $0 status
  $0 tail
Notes:
  • Uses Python file: $DEFAULT_PYFILE
  • Reads status file: $STATUS_FILE
  • Container '$NAME' must already be running.
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
    [[ -f "'"$STATUS_FILE"'" ]] || { echo "❌ Missing status file: '"$STATUS_FILE"'" >&2; exit 1; }
    [[ -f "'"$DEFAULT_PYFILE"'" ]] || { echo "❌ Missing Python file: '"$DEFAULT_PYFILE"'" >&2; exit 1; }

    source /opt/ros/humble/setup.bash
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true

    # Start detached; log to bags dir; record PID
    setsid python3 "'"$DEFAULT_PYFILE"'" --ros-args -p file_path:="'"$STATUS_FILE"'" '"${extra_args[@]+"${extra_args[@]}"}"' \
      > /home/dev/bags/file_state_publisher.log 2>&1 < /dev/null &
    echo $! > '"$PID_FILE"'
    echo "started"
  '
}

case "${1:-}" in
  start)
    shift
    require_running_container
    out=$(start_node "$@")
    [[ "$out" == "started" ]] && echo "✅ file_state_publisher_stamped started (logging to /home/dev/bags/file_state_publisher.log)"
    ;;
  stop)
    require_running_container
    echo "⏹ stopping…"
    docker exec "$NAME" bash -lc '
      if [[ -f "'"$PID_FILE"'" ]]; then
        kill -INT "$(cat "'"$PID_FILE"'")" 2>/dev/null || true
        rm -f "'"$PID_FILE"'"
      else
        pkill -INT -f "file_state_publisher.py|python3 .*file_state_publisher.py" 2>/dev/null || true
      fi
    '
    ;;
  status)
    require_running_container
    docker exec "$NAME" bash -lc '
      P1="file_state_publisher.py"
      if pgrep -af "$P1" >/dev/null; then
        echo "✅ file_state_publisher running:"
        pgrep -af "$P1"
      else
        echo "no file_state_publisher running"
      fi
      if [[ -f /home/dev/bags/file_state_publisher.log ]]; then
        echo; echo "Log tail:"
        tail -n 20 /home/dev/bags/file_state_publisher.log || true
      fi
    '
    ;;
  tail)
    require_running_container
    docker exec -it "$NAME" bash -lc '
      target=/home/dev/bags/file_state_publisher.log
      [[ -f "$target" ]] || { echo "no log file at $target"; exit 1; }
      echo "📄 Tailing $target (Ctrl+C to stop)"
      tail -n +1 -f "$target"
    '
    ;;
  *)
    usage; exit 1;;
esac


