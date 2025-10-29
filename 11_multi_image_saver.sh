#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/image_saver.pid
LOG_FILE=/home/dev/bags/image_saver.log
PYFILE=${PYFILE:-/home/dev/ws/src/owl_sense/owl_sense/image_saver.py}

usage(){ cat <<USAGE
Usage:
  $0 start [node-args-and-ros-args]
     examples:
       $0 start                   # no params
       $0 start --ros-args -p min_distance:=0.5
       $0 start --ros-args -p min_distance:=0.6 -p save_raw:=true
  $0 stop
  $0 status
  $0 tail
Notes:
  • Logs: $LOG_FILE
  • PID : $PID_FILE
USAGE
}

require_running_container(){
  docker ps --format '{{.Names}}' | grep -qx "$NAME" || {
    echo "❌ Container '$NAME' is not running. Start it first (e.g. ./01_start_container.sh)." >&2
    exit 1
  }
}

start_bg(){
  # pass every arg as a single env string (no filtering)
  local argstr="${*:-}"
  docker exec -e IMG_SAVER_ARGS="$argstr" -e PYFILE="$PYFILE" "$NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true
    mkdir -p /home/dev/bags

    # if already running, skip
    if [[ -f "'"$PID_FILE"'" ]] && kill -0 "$(cat "'"$PID_FILE"'")" 2>/dev/null; then
      echo "already_running"; exit 0
    fi

    # rebuild argv safely; our args have no embedded spaces, so word-splitting is fine
    cmd=( python3 -u "$PYFILE" )
    if [[ -n "${IMG_SAVER_ARGS:-}" ]]; then
      # shellcheck disable=SC2206
      extra=( ${IMG_SAVER_ARGS} )
      cmd+=("${extra[@]}")
    fi

    # show the exact command in the log header once
    {
      echo "========== image_saver start $(date -Iseconds) =========="
      printf "CMD:"; printf " %q" "${cmd[@]}"; echo
    } >> "'"$LOG_FILE"'"

    nohup "${cmd[@]}" >> "'"$LOG_FILE"'" 2>&1 < /dev/null &
    echo $! > "'"$PID_FILE"'"
    echo "started"
  '
}

stop_bg(){
  docker exec "$NAME" bash -lc '
    if [[ -f "'"$PID_FILE"'" ]]; then
      pid=$(cat "'"$PID_FILE"'")
      kill -INT "$pid" 2>/dev/null || true
      for _ in {1..50}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
      kill -TERM "$pid" 2>/dev/null || true
      for _ in {1..20}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
      kill -KILL "$pid" 2>/dev/null || true
      rm -f "'"$PID_FILE"'"
    else
      pkill -INT -f "owl_sense.*image_saver\.py" 2>/dev/null || true
    fi
  '
}

status_bg(){
  docker exec "$NAME" bash -lc '
    if [[ -f "'"$PID_FILE"'" ]] && kill -0 "$(cat "'"$PID_FILE"'")" 2>/dev/null; then
      echo "✅ image_saver running (pid $(cat "'"$PID_FILE"'"))"
    elif pgrep -af "owl_sense.*image_saver\.py" >/dev/null; then
      echo "⚠️ running (found by pattern), but PID file missing"
    else
      echo "no image_saver running"
    fi
    [[ -f "'"$LOG_FILE"'" ]] && { echo; echo "Log tail:"; tail -n 20 "'"$LOG_FILE"'"; }
  '
}

tail_bg(){
  require_running_container
  docker exec -it "$NAME" bash -lc '
    [[ -f "'"$LOG_FILE"'" ]] || { echo "no log file at '"$LOG_FILE"'"; exit 1; }
    echo "📄 Tailing '"$LOG_FILE"' (Ctrl+C to stop)"
    tail -n +1 -f "'"$LOG_FILE"'"
  '
}

case "${1:-}" in
  start)  shift || true; require_running_container; out=$(start_bg "$@"); [[ "$out" == "already_running" ]] && echo "ℹ️ already running" || echo "✅ started (log: $LOG_FILE)";;
  stop)   require_running_container; echo "⏹ stopping image_saver…"; stop_bg;;
  status) require_running_container; status_bg;;
  tail)   tail_bg;;
  *)      usage; exit 1;;
esac

