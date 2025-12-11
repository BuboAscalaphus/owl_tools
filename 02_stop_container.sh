#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/config.sh"

# Then stop/remove the container
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker stop -t 10 "$NAME" >/dev/null || true
  docker rm -f "$NAME" >/dev/null || true
  echo "🛑 Container '$NAME' stopped and removed."
else
  echo "ℹ️  Container '$NAME' not found."
fi

