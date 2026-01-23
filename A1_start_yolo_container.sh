#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh" 2>/dev/null || true

# Path del tuo app.py sul host (da montare nel container)
APP_PY="${WS}/ultra_api/app.py"
if [[ ! -f "$APP_PY" ]]; then
  echo "ERROR: app.py not found at: $APP_PY"
  echo "Fix APP_PY in this script to the correct path."
  exit 1
fi

docker rm -f yolo >/dev/null 2>&1 || true

docker run -d --name yolo \
  --network host \
  --runtime nvidia --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v "$WS/src/owl_bags:/home/dev/bags:ro" \
  -v "$WS/src/owl_weights:/models:ro" \
  -v "$APP_PY:/app/app.py" \
  yolo-api:latest \
  uvicorn app:app --host 0.0.0.0 --port 8000 --reload

# Quick peek at logs
docker logs --tail 80 yolo || true
echo "✅ YOLO API container 'yolo' started with app.py bind-mounted + uvicorn --reload."
echo "   Host app.py: $APP_PY"
