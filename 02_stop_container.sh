#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"

# Try to stop rosbag nicely (won't error if nothing running)
"$(dirname "$0")/09_record_bag.sh" stop || true

# Then stop/remove the container
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker stop -t 10 "$NAME" >/dev/null || true
  docker rm -f "$NAME" >/dev/null || true
  echo "🛑 Container '$NAME' stopped and removed."
else
  echo "ℹ️  Container '$NAME' not found."
fi

