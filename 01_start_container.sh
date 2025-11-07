#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh" 2>/dev/null || true

# Use a name that won't clash with host env
CONTAINER_USER="${CONTAINER_USER:-dev}"

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
# ----- Devices: only matching Camer filter (case insensitive) -----
DEVICES=()
for n in /sys/class/video4linux/*; do
  [[ -e "$n/name" ]] || continue
  if grep -qi "${CAMERA_FILTER}" "$n/name"; then
    DEVICES+=( --device "/dev/$(basename "$n")" )
  fi
done
for n in /sys/class/v4l-subdev/*; do
  [[ -e "$n/name" ]] || continue
  if grep -qi "${CAMERA_FILTER}" "$n/name"; then
    DEVICES+=( --device "/dev/$(basename "$n")" )
  fi
done


if [[ ${#DEVICES[@]} -eq 0 ]]; then
  echo "⚠️  No matching cameras found. Starting container without cameras."
fi


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

# If container exists, ensure /home/$CONTAINER_USER/ws is mounted; otherwise recreate
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  if docker inspect -f "{{range .Mounts}}{{if eq .Destination \"/home/${CONTAINER_USER}/ws\"}}OK{{end}}{{end}}" "$NAME" | grep -q 'OK'; then
    docker start "$NAME" >/dev/null
    echo "✅ Container '$NAME' started (workspace mount present)."
    exit 0
  else
    echo "⚠️  Container '$NAME' exists but /home/${CONTAINER_USER}/ws is NOT mounted. Recreating..."
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
  -v "$WS":/home/"$CONTAINER_USER"/ws \
  -v "$HOME/.gitconfig":/home/"$CONTAINER_USER"/.gitconfig:ro \
  -v "$HOME/.git-credentials":/home/"$CONTAINER_USER"/.git-credentials:ro \
  -v "$HOME/.gitconfig":/root/.gitconfig:ro \
  -v "$HOME/.git-credentials":/root/.git-credentials:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$BAGS":/home/"$CONTAINER_USER"/bags \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e TZ="$(cat /etc/timezone)" \
  "${VIZION_ARGS[@]}" \
  "${NV_ARGS[@]}" \
  "$IMAGE" sleep infinity

echo "✅ Container '$NAME' created and running."



