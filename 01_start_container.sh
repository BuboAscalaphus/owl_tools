#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

# Build devices list
DEVICES=$(for d in /dev/video* /dev/media*; do [[ -e "$d" ]] && printf -- "--device %s " "$d"; done)

# Optional NV plugin mounts/env
NV_ARGS=""
if [[ "$NV" == "1" ]]; then
  NV_ARGS+=" -e LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu "
  NV_ARGS+=" -e GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0 "
  NV_ARGS+=" -e GST_PLUGIN_SCANNER=/usr/lib/aarch64-linux-gnu/gstreamer-1.0/gst-plugin-scanner "
  NV_ARGS+=" -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro "
  NV_ARGS+=" -v /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl:ro "
  NV_ARGS+=" -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/gstreamer-1.0:ro "
fi

mkdir -p "$WS"

# If container exists, just start it
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker start "$NAME" >/dev/null
  echo "✅ Container '$NAME' started."
  exit 0
fi

# Allow X11 (for rqt/RViz)
xhost +local:root >/dev/null 2>&1 || true

# Run persistent container
docker run -d --name "$NAME" \
  --network host --ipc=host --shm-size=1g \
  --runtime nvidia --gpus all \
  $DEVICES \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -v "$WS":/home/dev/ws \
  -v "$HOME/.gitconfig":/home/dev/.gitconfig:ro \
  -v "$HOME/.git-credentials":/home/dev/.git-credentials:ro \
  -v "$HOME/.gitconfig":/root/.gitconfig:ro \
  -v "$HOME/.git-credentials":/root/.git-credentials:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$BAGS":/home/dev/bags \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e TZ="$(cat /etc/timezone)" \
  $NV_ARGS \
  "$IMAGE" sleep infinity

echo "✅ Container '$NAME' created and running."
