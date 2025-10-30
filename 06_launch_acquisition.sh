#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/acquisition.pid       # PID of the "ros2 launch" process
PGID_FILE=/tmp/acquisition.pgid     # Process Group ID for group signaling
LOG_FILE=/home/dev/bags/acquisition.log

usage(){ cat <<USAGE
Usage:
  $0 start [extra-ros-args]
  $0 stop
  $0 status
  $0 tail
Notes:
  • Logs to: $LOG_FILE
  • Container '$NAME' must be running.
USAGE
}

require_running_container(){
  docker ps --format '{{.Names}}' | grep -qx "$NAME" || {
    echo "❌ Container '$NAME' is not running. Start it first (e.g. ./01_start_container.sh)." >&2
    exit 1
  }
}

start_bg(){
  # pass extra args safely via env to avoid host-side quoting issues
  local argstr="${*:-}"
  docker exec -e ARG_STR="$argstr" "$NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true
    mkdir -p /home/dev/bags

    # Avoid double start
    if [[ -f "'"$PID_FILE"'" ]] && kill -0 "$(cat "'"$PID_FILE"'")" 2>/dev/null; then
      echo "already_running"; exit 0
    fi

    # Start in a NEW SESSION so we can later signal the whole group (pgid)
    # "exec" makes ros2 replace the shell; $! will be its PID
    setsid bash -lc "exec ros2 launch owl_sense acquisition.launch.py $ARG_STR" >> "'"$LOG_FILE"'" 2>&1 &
    pid=$!
    echo "$pid" > "'"$PID_FILE"'"

    # Record the process group id (normally equals pid for a new session)
    pgid=$(ps -o pgid= -p "$pid" | tr -d " ")
    [[ -n "$pgid" ]] && echo "$pgid" > "'"$PGID_FILE"'"

    echo "started"
  '
}

stop_bg(){
  docker exec "$NAME" bash -lc '
    set -e
    PID_FILE="'"$PID_FILE"'"
    PGID_FILE="'"$PGID_FILE"'"

    get_target() {
      if [[ -f "$PGID_FILE" ]]; then
        pgid=$(cat "$PGID_FILE"); [[ -n "$pgid" ]] && { echo "-$pgid"; return; }
      fi
      if [[ -f "$PID_FILE" ]]; then cat "$PID_FILE"; return; fi
      echo ""
    }

    send_and_wait() {
      sig="$1"; target="$2"; timeout="${3:-5}"
      kill -"${sig}" ${target} 2>/dev/null || return 1
      # For kill -0, strip a leading '-' (group) to probe the leader PID
      probe=${target#-}
      for _ in $(seq 1 $((timeout*10))); do
        kill -0 "$probe" 2>/dev/null || return 0
        sleep 0.1
      done
      return 1
    }

    target="$(get_target)"
    if [[ -n "$target" ]]; then
      send_and_wait INT  "$target" 5 || \
      send_and_wait TERM "$target" 5 || \
      send_and_wait KILL "$target" 2 || true
    else
      # Fallback if no pid/pgid found
      pkill -INT -f "ros2 launch .*owl_sense.*acquisition" 2>/dev/null || true
    fi

    rm -f "$PID_FILE" "$PGID_FILE" || true
    # final cleanup (harmless if nothing remains)
    pkill -TERM -f "ros2 launch .*owl_sense.*acquisition" 2>/dev/null || true
  '
}

status_bg(){
  docker exec "$NAME" bash -lc '
    PID_FILE="'"$PID_FILE"'"; LOG_FILE="'"$LOG_FILE"'"
    running=0
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "✅ acquisition running (pid $(cat "$PID_FILE"))"; running=1
    elif pgrep -af "ros2 launch .*owl_sense.*acquisition" >/dev/null; then
      echo "⚠️ acquisition seems running (found by pattern), but PID file is missing."; running=1
    else
      echo "no acquisition running"
    fi

    if [[ -f "$LOG_FILE" ]]; then
      echo; echo "Log tail ($LOG_FILE):"; tail -n 20 "$LOG_FILE"
    elif [[ $running -eq 1 ]]; then
      echo; echo "ℹ️ No log file yet at $LOG_FILE."
    fi
  '
}

tail_bg(){
  docker exec -it "$NAME" bash -lc '
    [[ -f "'"$LOG_FILE"'" ]] || { echo "no log file at '"$LOG_FILE"'"; exit 1; }
    echo "📄 Tailing '"$LOG_FILE"' (Ctrl+C to stop)"
    tail -n +1 -f "'"$LOG_FILE"'"
  '
}

case "${1:-}" in
  start)
    shift || true
    require_running_container
    out=$(start_bg "$@")
    [[ "$out" == "already_running" ]] && echo "ℹ️ acquisition already running (see: $0 status / $0 tail)" || echo "✅ acquisition started (logging to $LOG_FILE)"
    ;;
  stop)
    require_running_container
    echo "⏹ stopping acquisition…"
    stop_bg
    ;;
  status)
    require_running_container
    status_bg
    ;;
  tail)
    require_running_container
    tail_bg
    ;;
  *)
    usage; exit 1;;
esac


