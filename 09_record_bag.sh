#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

PID_FILE=/tmp/rosbag2_record.pid

usage(){ cat <<USAGE
Usage:
  $0 start <topic1> [topic2 ...]
  $0 start-all
  $0 stop
  $0 status
  $0 tail
  $0 list
USAGE
}

prefill_perms(){
  docker exec -u root "$NAME" bash -lc 'mkdir -p /home/dev/bags && chown -R dev:dev /home/dev/bags'
}

start_all(){
  docker exec "$NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    ts=$(date +%Y%m%d_%H%M%S)
    bagdir=/home/dev/bags/$ts      # must not exist yet
    logfile=/home/dev/bags/${ts}.log
    # start detached; PID is the real recorder
    setsid ros2 bag record -s sqlite3 -a -o "$bagdir" >"$logfile" 2>&1 < /dev/null &
    echo $! > "'"$PID_FILE"'"
    # wait for dir and wire helpers
    for i in {1..50}; do
      [[ -d "$bagdir" ]] && { ln -sfn "$bagdir" /home/dev/bags/latest; ln -sfn "$logfile" "$bagdir/record.log" || true; break; }
      sleep 0.1
    done
    echo "started:$ts"
  '
}

start_some(){
  # pass topics via env to avoid quoting hell
  local topics="$*"
  docker exec -e TOPICS="$topics" "$NAME" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    ts=$(date +%Y%m%d_%H%M%S)
    bagdir=/home/dev/bags/$ts
    logfile=/home/dev/bags/${ts}.log
    setsid ros2 bag record -s sqlite3 -o "$bagdir" $TOPICS >"$logfile" 2>&1 < /dev/null &
    echo $! > "'"$PID_FILE"'"
    for i in {1..50}; do
      [[ -d "$bagdir" ]] && { ln -sfn "$bagdir" /home/dev/bags/latest; ln -sfn "$logfile" "$bagdir/record.log" || true; break; }
      sleep 0.1
    done
    echo "started:$ts"
  '
}

case "${1:-}" in
  start)
    shift; [[ $# -ge 1 ]] || { echo "❌ provide at least one topic"; usage; exit 1; }
    prefill_perms
    out=$(start_some "$@")
    ts=${out#started:}
    echo "✅ Recording to host: $BAGS/$ts"
    ;;
  start-all)
    prefill_perms
    out=$(start_all)
    ts=${out#started:}
    echo "✅ Recording ALL topics to host: $BAGS/$ts"
    ;;
  stop)
    echo "⏹ stopping…"
    docker exec "$NAME" bash -lc '
      if [[ -f "'"$PID_FILE"'" ]]; then
        kill -INT "$(cat "'"$PID_FILE"'")" 2>/dev/null || true
        rm -f "'"$PID_FILE"'"
      else
        pkill -INT -f "ros2bag.*record|ros2 bag record|python3 .*ros2bag.*record" 2>/dev/null || true
      fi
    '
    ;;
  status)
    docker exec "$NAME" bash -lc '
      P1="ros2 bag record"; P2="ros2bag.*record"; P3="python3 .*ros2bag.*record"
      if pgrep -af "$P1|$P2|$P3" >/dev/null; then
        echo "✅ rosbag2 recording is running:"
        pgrep -af "$P1|$P2|$P3"
      else
        echo "no rosbag2 recording running"
      fi
      if [[ -e /home/dev/bags/latest ]]; then
        echo; echo "Latest bag:";
        ls -ld /home/dev/bags/latest
        du -sh /home/dev/bags/latest 2>/dev/null || true
      fi
    '
    ;;
  tail)
    docker exec -it "$NAME" bash -lc '
      target=/home/dev/bags/latest/record.log
      if [[ ! -f "$target" ]]; then
        latest_log=$(ls -1t /home/dev/bags/*.log 2>/dev/null | head -1 || true)
        [[ -n "$latest_log" ]] || { echo "no recorder log found"; exit 1; }
        target="$latest_log"
      fi
      echo "📄 Tailing $target (Ctrl+C to stop)"
      tail -n +1 -f "$target"
    '
    ;;
  list)
    ls -1 "$BAGS" 2>/dev/null || echo "no bags in $BAGS"
    ;;
  *)
    usage; exit 1;;
esac

