#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
source "$HERE/config.sh"

ACQ=./06_launch_acquisition.sh
IMG=./11_multi_image_saver.sh
STATE=./10_file_state_publisher.sh
START_CONTAINER=./01_start_container.sh

# 1) container
"$START_CONTAINER"

# 2) acquisition (bg; logs handled in 06)
"$ACQ" start

# 3) image_saver with min_distance=0.5
"$IMG" start --ros-args -p min_distance:=1.8

# 3b) wait briefly so the first state msg isn’t missed
echo "⏳ waiting for image_saver…"; sleep 2

# 4) file state publisher
"$STATE" start

echo "🎯 Pipeline started. Use:"
echo "  $ACQ tail"
echo "  $IMG tail"
echo "  $STATE tail"

