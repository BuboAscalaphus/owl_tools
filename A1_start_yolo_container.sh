#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh" 2>/dev/null || true

docker rm -f yolo >/dev/null 2>&1 || true

docker run -d --name yolo \
  --network host \
  --runtime nvidia --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v "$HOME/$WS_DIR_NAME/src/owl_bags:/home/dev/bags:ro" \
  -v "$HOME/$WS_DIR_NAME/src/owl_weights:/models:ro" \
  yolo-api:latest
  
# Quick peek at logs in case the image prints where it’s listening
docker logs --tail 80 yolo || true
echo "✅ YOLO API container 'yolo' created; using image's default entrypoint."


