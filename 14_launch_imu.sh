#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/config.sh"   # Deve definire almeno $NAME (nome del container)

# --- Costanti e file di stato (dentro il container) ---
# Nomi univoci per la IMU
PID_FILE=/tmp/imu.pid
PGID_FILE=/tmp/imu.pgid
LOG_FILE=/home/dev/bags/imu.log

PKG_NAME="owl_imu"
# Assumo che il file sia dentro la cartella "launch/" del pacchetto.
# Se è nella root del pacchetto, cambia in "launch.imu.py"
LAUNCH_REL="launch/imu.launch.py" 

usage() {
  cat <<USAGE
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

require_running_container() {
  docker ps --format '{{.Names}}' | grep -qx "$NAME" || {
    echo "❌ Container '$NAME' is not running. Start it first." >&2
    exit 1
  }
}

start_bg() {
  docker exec \
    -e PID_FILE="$PID_FILE" \
    -e PGID_FILE="$PGID_FILE" \
    -e LOG_FILE="$LOG_FILE" \
    -e PKG_NAME="$PKG_NAME" \
    -e LAUNCH_REL="$LAUNCH_REL" \
    -e EXTRA_ARGS="$*" \
    "$NAME" bash -lc '
      set -euo pipefail
      
      # ... (parte di source ROS identica) ...
      set +u
      source /opt/ros/humble/setup.bash || exit 1
      [ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash" || true
      set -u

      mkdir -p "$(dirname "$LOG_FILE")"

      # ... (check already running identico) ...
      if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "already_running"
        exit 0
      fi

      # ... (risoluzione path identica) ...
      PKG_PREFIX="$(ros2 pkg prefix "$PKG_NAME" 2>/dev/null || true)"
      if [[ -n "$PKG_PREFIX" ]] && [[ -f "$PKG_PREFIX/share/$PKG_NAME/$LAUNCH_REL" ]]; then
        LAUNCH_FILE="$PKG_PREFIX/share/$PKG_NAME/$LAUNCH_REL"
      elif [[ -f "$HOME/ws/src/$PKG_NAME/$LAUNCH_REL" ]]; then
        LAUNCH_FILE="$HOME/ws/src/$PKG_NAME/$LAUNCH_REL"
      fi

      if [[ -z "$LAUNCH_FILE" ]]; then echo "❌ File not found"; exit 1; fi

      {
        echo "===== $(date -Is) ====="
        echo "LAUNCH_FILE : $LAUNCH_FILE"
      } >> "$LOG_FILE"

      # --- MODIFICA QUI ---
      export PYTHONUNBUFFERED=1
      export RCUTILS_LOGGING_BUFFERED_STREAM=1
      # --------------------

      setsid bash -lc "exec ros2 launch \"$LAUNCH_FILE\" $EXTRA_ARGS" \
        >> "$LOG_FILE" 2>&1 &

      pid=$!
      echo "$pid" > "$PID_FILE"
      pgid="$(ps -o pgid= -p "$pid" | tr -d " ")"
      [[ -n "$pgid" ]] && echo "$pgid" > "$PGID_FILE"
      echo "started"
    '
}

stop_bg() {
  docker exec \
    -e PID_FILE="$PID_FILE" \
    -e PGID_FILE="$PGID_FILE" \
    "$NAME" bash -lc '
      set -euo pipefail

      get_target() {
        if [[ -f "$PGID_FILE" ]]; then
          pgid="$(cat "$PGID_FILE" 2>/dev/null || true)"
          [[ -n "$pgid" ]] && { echo "-$pgid"; return; }
        fi
        if [[ -f "$PID_FILE" ]]; then
          cat "$PID_FILE"
          return
        fi
        echo ""
      }

      send_and_wait() {
        local sig="$1"
        local target="$2"
        local timeout="${3:-5}"
        kill -"${sig}" ${target} 2>/dev/null || return 1
        local probe="${target#-}"
        for _ in $(seq 1 $((timeout * 10))); do
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
        # Fallback specifico per imu (cerca owl_imu nel comando)
        pkill -INT -f "ros2 launch .*owl_imu.*" 2>/dev/null || true
      fi

      rm -f "$PID_FILE" "$PGID_FILE" || true
      pkill -TERM -f "ros2 launch .*owl_imu.*" 2>/dev/null || true
    '
}

status_bg() {
  docker exec \
    -e PID_FILE="$PID_FILE" \
    -e LOG_FILE="$LOG_FILE" \
    "$NAME" bash -lc '
      set -euo pipefail

      running=0
      if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "✅ IMU running (pid $(cat "$PID_FILE"))"
        running=1
      elif pgrep -af "ros2 launch .*owl_imu.*" >/dev/null; then
        echo "⚠️ IMU seems running (found by pattern), but PID file is missing."
        running=1
      else
        echo "no IMU running"
      fi

      if [[ -f "$LOG_FILE" ]]; then
        echo
        echo "Log tail ($LOG_FILE):"
        tail -n 20 "$LOG_FILE"
      elif [[ $running -eq 1 ]]; then
        echo
        echo "ℹ️ No log file yet at $LOG_FILE."
      fi
    '
}

tail_bg() {
  docker exec -it \
    -e LOG_FILE="$LOG_FILE" \
    "$NAME" bash -lc '
      [[ -f "$LOG_FILE" ]] || { echo "no log file at $LOG_FILE"; exit 1; }
      echo "📄 Tailing $LOG_FILE (Ctrl+C to stop)"
      tail -n +1 -f "$LOG_FILE"
    '
}

case "${1:-}" in
  start)
    shift || true
    require_running_container
    out="$(start_bg "$@")"
    if [[ "$out" == "already_running" ]]; then
      echo "ℹ️ IMU already running (see: $0 status / $0 tail)"
    else
      echo "✅ IMU started (logging to $LOG_FILE)"
    fi
    ;;
  stop)
    require_running_container
    echo "⏹ stopping IMU…"
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
    usage
    exit 1
    ;;
esac
