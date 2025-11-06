#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
# Optional config with NAME=... (container name)
source "$HERE/config.sh" 2>/dev/null || true

NAME="${NAME:-owl-dev}"
PID_FILE=/tmp/processing.pid
LOG_HINT=/home/dev/bags/processing.log            # preferred location
LOG_PATH_FILE=/tmp/processing.log.path            # where we store the actual log path used
LAUNCH_PKG=owl_image_queue
LAUNCH_FILE=orchard_processing.launch.py

usage(){ cat <<USAGE
Usage:
  $0 start
  $0 stop
  $0 status
  $0 tail
Notes:
  • Container '$NAME' must be running.
  • Launch: ros2 launch $LAUNCH_PKG $LAUNCH_FILE
USAGE
}

require_running(){ docker ps --format '{{.Names}}' | grep -qx "$NAME" || { echo "❌ Container '$NAME' is not running."; exit 1; }; }

start_cmd() {
  require_running

  # Already running?
  if docker exec "$NAME" bash -lc '[ -f "/tmp/processing.pid" ] && kill -0 $(cat /tmp/processing.pid) 2>/dev/null'; then
    echo "ℹ️ processing already running (see: $0 status / $0 tail)"; return 0
  fi

  # Pre-create log and start in a new session immune to SIGHUP
  docker exec "$NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    [ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash" || true
    mkdir -p /home/dev/bags
    : > /home/dev/bags/processing.log
    nohup setsid ros2 launch owl_image_queue orchard_processing.launch.py >> /home/dev/bags/processing.log 2>&1 &
    echo $! > /tmp/processing.pid
  '

  # Health check (up to ~3s)
  for i in {1..30}; do
    sleep 0.1
    if docker exec "$NAME" bash -lc 'kill -0 $(cat /tmp/processing.pid) 2>/dev/null'; then
      echo "✅ processing started (logging to /home/dev/bags/processing.log)"
      return 0
    fi
  done

  echo "❌ processing failed to start. Check: /home/dev/bags/processing.log and ~/.ros/log"
  exit 1
}

stop_cmd() {
  require_running
  docker exec "$NAME" bash -lc '
    set -e
    PID_FILE="'"$PID_FILE"'"
    if [ -f "$PID_FILE" ]; then
      pid=$(cat "$PID_FILE")
      kill -INT "$pid" 2>/dev/null || true
      for i in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || { rm -f "$PID_FILE"; echo "✅ processing stopped"; exit 0; }
        sleep 0.1
      done
      kill -KILL "$pid" 2>/dev/null || true
      rm -f "$PID_FILE"
      echo "✅ processing force-stopped"
    else
      echo "no processing pid file"; exit 0
    fi
  '
}

status_cmd() {
  require_running
  docker exec "$NAME" bash -lc '
    PID_FILE="'"$PID_FILE"'"
    LOG_PATH_FILE="'"$LOG_PATH_FILE"'"
    LOG=$(cat "$LOG_PATH_FILE" 2>/dev/null || echo "'"$LOG_HINT"'")
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "✅ processing running (pid $(cat "$PID_FILE"))"
    else
      echo "no processing running"
    fi
    echo
    if [ -f "$LOG" ]; then
      echo "Log tail ($LOG):"
      tail -n 20 "$LOG"
    else
      echo "ℹ️ No log file yet at $LOG."
    fi
  '
}

tail_cmd() {
  require_running
  # resolve actual log path first
  LOG_ACTUAL=$(docker exec "$NAME" bash -lc 'cat "'"$LOG_PATH_FILE"'" 2>/dev/null || echo "'"$LOG_HINT"'"')
  docker exec -it "$NAME" bash -lc "
    if [ ! -f '$LOG_ACTUAL' ]; then
      echo 'no log file at $LOG_ACTUAL'; exit 1
    fi
    echo '📄 Tailing $LOG_ACTUAL (Ctrl+C to stop)'
    tail -n +1 -f '$LOG_ACTUAL'
  "
}

case "${1:-}" in
  start)  start_cmd ;;
  stop)   stop_cmd ;;
  status) status_cmd ;;
  tail)   tail_cmd ;;
  *)      usage; exit 1 ;;
esac

