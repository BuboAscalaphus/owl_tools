#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

# Optional env:
#   PRIV=1                    -> add --privileged -v /dev:/dev
#   VIZION_CFG=/etc/vizion   -> bind this path & export inside container
PRIV="${PRIV:-0}"
VIZION_CFG="${VIZION_CFG:-/etc/vizion}"

# ----- Groups (host numeric GIDs) -----
gid_or_empty() { getent group "$1" | cut -d: -f3 || true; }
GID_VIDEO="$(gid_or_empty video)"
GID_I2C="$(gid_or_empty i2c || true)"
GID_PLUGDEV="$(gid_or_empty plugdev || true)"

GROUP_ARGS=()
[[ -n "${GID_VIDEO}"   ]] && GROUP_ARGS+=( --group-add "${GID_VIDEO}" )
[[ -n "${GID_I2C}"     ]] && GROUP_ARGS+=( --group-add "${GID_I2C}" )
[[ -n "${GID_PLUGDEV}" ]] && GROUP_ARGS+=( --group-add "${GID_PLUGDEV}" )

# ----- Devices (video/media/subdev/i2c) -----
DEVICES=()
for d in /dev/video* /dev/media* /dev/v4l-subdev* /dev/i2c-*; do
  [[ -e "$d" ]] && DEVICES+=( --device "$d" )
done

# ----- USB/sysfs/udev & cgroup rules (as array to avoid quote issues) -----
# ----- USB/sysfs/udev & cgroup rules (each value must be ONE arg) -----
USB_ARGS=(
  "--device-cgroup-rule=c 189:* rmw"
  "--device-cgroup-rule=c 81:* rmw"
  "-v" "/dev/bus/usb:/dev/bus/usb"
  "-v" "/sys/bus/usb:/sys/bus/usb:ro"
  "-v" "/sys/devices:/sys/devices:ro"
  "-v" "/run/udev:/run/udev:ro"
)
# ----- NV plugin mounts/env (optional) -----
NV_ARGS=()
if [[ "${NV:-0}" == "1" ]]; then
  NV_ARGS+=(
    -e LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu
    -e GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
    -e GST_PLUGIN_SCANNER=/usr/lib/aarch64-linux-gnu/gstreamer-1.0/gst-plugin-scanner
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro
    -v /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl:ro
    -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/gstreamer-1.0:ro
  )
fi

# ----- Vizion config mount (optional) -----
VIZION_ARGS=()
if [[ -d "$VIZION_CFG" ]]; then
  VIZION_ARGS+=( -e "VIZIONSDK_CONFIG_PATH=$VIZION_CFG" -v "$VIZION_CFG:$VIZION_CFG:ro" )
fi

mkdir -p "$WS"

# If container exists, ensure /home/dev/ws is mounted; otherwise recreate
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  if docker inspect -f '{{range .Mounts}}{{if eq .Destination "/home/dev/ws"}}OK{{end}}{{end}}' "$NAME" | grep -q 'OK'; then
    docker start "$NAME" >/dev/null
    echo "✅ Container '$NAME' started (workspace mount present)."
    exit 0
  else
    echo "⚠️  Container '$NAME' exists but /home/dev/ws is NOT mounted. Recreating..."
    docker rm -f "$NAME" >/dev/null
  fi
fi

# Allow X11
xhost +local:root >/dev/null 2>&1 || true

# Optional privileged (diagnostic)
PRIV_ARGS=()
if [[ "$PRIV" == "1" ]]; then
  PRIV_ARGS+=( --privileged -v /dev:/dev )
fi

# Run persistent container
docker run -d --name "$NAME" \
  --network host --ipc=host --shm-size=1g \
  --runtime nvidia --gpus all \
  "${PRIV_ARGS[@]}" \
  "${DEVICES[@]}" \
  "${USB_ARGS[@]}" \
  "${GROUP_ARGS[@]}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
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
  "${VIZION_ARGS[@]}" \
  "${NV_ARGS[@]}" \
  "$IMAGE" sleep infinity

echo "✅ Container '$NAME' created and running."

# YOLO sidecar
if ! docker ps -a --format '{{.Names}}' | grep -qx "yolo"; then
  docker run -d --name yolo \
    --network host \
    --runtime nvidia --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$BAGS":/home/dev/bags:ro \
    yolo-api:latest
  echo "✅ YOLO API container 'yolo' created and running (host net)."
else
  docker start yolo >/dev/null
  echo "✅ YOLO API container 'yolo' started."
fi

