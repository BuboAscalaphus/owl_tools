#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/image_saver.pid
LOG_FILE=/home/dev/bags/image_saver.log
DEFAULT_PYFILE=/home/dev/ws/src/owl_sense/owl_sense/image_saver.py
DEFAULT_BASE_DIR=/home/dev/bags   # images will be saved under here

usage(){ cat <<USAGE
Usage:
  $0 start [extra-ros-args]
  $0 stop
  $0 status
  $0 tail
Notes:
  • Uses Python file: $DEFAULT_PYFILE
  • Saves under:      $DEFAULT_BASE_DIR
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
    [[ -f "'"$DEFAULT_PYFILE"'" ]] || { echo "❌ Missing Python file: '"$DEFAULT_PYFILE"'" >&2; exit 1; }
    mkdir -p "'"$DEFAULT_BASE_DIR"'"

    source /opt/ros/humble/setup.bash
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true

    # Start detached; log to bags dir; record PID
    setsid python3 "'"$DEFAULT_PYFILE"'" --ros-args \
      -p base_dir:="'"$DEFAULT_BASE_DIR"'" \
      '"${extra_args[@]+"${extra_args[@]}"}"' \
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
    [[ "$out" == "started" ]] && echo "✅ image_saver started (logging to $LOG_FILE)"
    ;;
  stop)
    require_running_container
    echo "⏹ stopping…"
    docker exec "$NAME" bash -lc '
      if [[ -f "'"$PID_FILE"'" ]]; then
        kill -INT "$(cat "'"$PID_FILE"'")" 2>/dev/null || true
        rm -f "'"$PID_FILE"'"
      else
        pkill -INT -f "image_saver.py|python3 .*image_saver.py" 2>/dev/null || true
      fi
    '
    ;;
  status)
    require_running_container
    docker exec "$NAME" bash -lc '
      P1="image_saver.py"
      if pgrep -af "$P1" >/dev/null; then
        echo "✅ image_saver running:"
        pgrep -af "$P1"
      else
        echo "no image_saver running"
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

