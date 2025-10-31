#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/acquisition.pid
PGID_FILE=/tmp/acquisition.pgid
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
  local argstr="${*:-}"
  docker exec -e ARG_STR="$argstr" "$NAME" bash -lc '
    # Source ROS without nounset tripping
    set -eo pipefail
    export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
    source /opt/ros/humble/setup.bash || { echo "Failed to source /opt/ros/humble" >&2; exit 1; }
    [ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash || true
    set -euo pipefail

    mkdir -p /home/dev/bags

    # Avoid double start
    if [[ -f "'"$PID_FILE"'" ]] && kill -0 "$(cat "'"$PID_FILE"'")" 2>/dev/null; then
      echo "already_running"; exit 0
    fi

    # Resolve launch + YAML
    PKG_PREFIX="$(ros2 pkg prefix owl_sense 2>/dev/null || true)"

    INST_LAUNCH="${PKG_PREFIX:+$PKG_PREFIX/share/owl_sense/launch/acquisition.launch.py}"
    SRC_LAUNCH="$HOME/ws/src/owl_sense/launch/acquisition.launch.py"

    # Preferred YAML: installed ar082.params.yaml → fallback to src
    INST_CFG="${PKG_PREFIX:+$PKG_PREFIX/share/owl_sense/config/cameras/ar082.params.yaml}"
    SRC_CFG="$HOME/ws/src/owl_sense/config/cameras/ar082.params.yaml"

    LAUNCH_FILE=""
    CONFIG_FILE=""

    [[ -n "${INST_LAUNCH:-}" && -f "$INST_LAUNCH" ]] && LAUNCH_FILE="$INST_LAUNCH"
    [[ -z "$LAUNCH_FILE" && -f "$SRC_LAUNCH" ]] && LAUNCH_FILE="$SRC_LAUNCH"

    [[ -n "${INST_CFG:-}" && -f "$INST_CFG" ]] && CONFIG_FILE="$INST_CFG"
    [[ -z "$CONFIG_FILE" && -f "$SRC_CFG" ]] && CONFIG_FILE="$SRC_CFG"

    if [[ -z "$LAUNCH_FILE" ]]; then
      echo "❌ acquisition.launch.py not found. Checked: $INST_LAUNCH ; $SRC_LAUNCH" >&2
      exit 1
    fi
    if [[ -z "$CONFIG_FILE" ]]; then
      echo "❌ ar082.params.yaml not found. Checked: $INST_CFG ; $SRC_CFG" >&2
      exit 1
    fi

    # Detect the correct launch argument name
    SHOW_ARGS="$(ros2 launch "$LAUNCH_FILE" --show-args 2>/dev/null || true)"
    # Common names used in ROS2 launch files
    CANDIDATES=(config params param_file config_file yaml_file)
    ARG_KEY=""
    for k in "${CANDIDATES[@]}"; do
      if printf "%s" "$SHOW_ARGS" | grep -q -E -- "--$k([ =]|$)"; then
        ARG_KEY="$k"
        break
      fi
    done
    # Fallback: if nothing matched, try the legacy default "config"
    ARG_KEY="${ARG_KEY:-config}"

    {
      echo "===== $(date -Is) ====="
      echo "LAUNCH_FILE: $LAUNCH_FILE"
      echo "CONFIG_FILE: $CONFIG_FILE"
      echo "ARG_KEY    : $ARG_KEY"
      echo "EXTRA_ARGS : $ARG_STR"
    } >> "'"$LOG_FILE"'"

    # Start in a new session; keep your PGID tracking
    setsid bash -lc "exec ros2 launch \"$LAUNCH_FILE\" $ARG_KEY:=\"$CONFIG_FILE\" $ARG_STR" >> "'"$LOG_FILE"'" 2>&1 &

    pid=$!
    echo "$pid" > "'"$PID_FILE"'"
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
      pkill -INT -f "ros2 launch .*owl_sense.*acquisition" 2>/dev/null || true
    fi

    rm -f "$PID_FILE" "$PGID_FILE" || true
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

