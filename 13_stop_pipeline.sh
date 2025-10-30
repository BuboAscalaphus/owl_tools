#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
source "$HERE/config.sh"

echo "⏹ Stopping pipeline (scripts + safety kill)…"

# 1) Try graceful stops via your component scripts (ignore errors if already stopped)
[[ -x ./10_file_state_publisher.sh ]] && ./10_file_state_publisher.sh stop || true
[[ -x ./11_multi_image_saver.sh    ]] && ./11_multi_image_saver.sh stop    || true
[[ -x ./06_launch_acquisition.sh   ]] && ./06_launch_acquisition.sh stop   || true
# If you sometimes record bags, stop that too (safe if the script isn't there)
[[ -x ./09_record_bag.sh           ]] && ./09_record_bag.sh stop           || true

# 2) If container isn't running, we're done
if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "✅ Container '$NAME' not running — nothing left to stop."
  exit 0
fi

# 3) Safety kill inside the container (INT → TERM → KILL) for known patterns
docker exec "$NAME" bash -lc '
set -e

patterns=(
  "owl_sense.*image_saver\.py"
  "ros2 launch .*owl_sense.*acquisition"
  "file_state_publisher"
  "ros2 bag record"
  "rqt"  # just in case you used 07_rqt.sh
)

kill_pattern() {
  local pat="$1"
  if pgrep -af "$pat" >/dev/null; then
    echo "  • stopping: $pat"
    pkill -INT  -f "$pat" 2>/dev/null || true
    for i in {1..25}; do pgrep -af "$pat" >/dev/null || return 0; sleep 0.2; done
    pkill -TERM -f "$pat" 2>/dev/null || true
    for i in {1..15}; do pgrep -af "$pat" >/dev/null || return 0; sleep 0.2; done
    pkill -KILL -f "$pat" 2>/dev/null || true
  fi
}

for pat in "${patterns[@]}"; do
  kill_pattern "$pat"
done

# 4) Clean up PID files so future starts are clean
rm -f /tmp/image_saver.pid \
      /tmp/acquisition.pid /tmp/acquisition.pgid \
      /tmp/file_state_publisher.pid \
      /tmp/rosbag2.pid 2>/dev/null || true

echo "  • verifying residual processes…"
leftover=$(pgrep -af "owl_sense|ros2 launch|file_state_publisher|ros2 bag record" || true)
if [[ -n "$leftover" ]]; then
  echo "    ⚠️ still found:"
  echo "$leftover"
else
  echo "    ✅ none found"
fi
'

echo "✅ Done."
echo "Tip: run './12_pipeline.sh status' to confirm everything is stopped."

