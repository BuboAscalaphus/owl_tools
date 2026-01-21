#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
source "$HERE/config.sh"

ACQ=./06_launch_acquisition.sh
ACQ2=./15_launch_rtk.sh
ACQ3=./14_launch_imu.sh
IMG=./11_multi_image_saver.sh
PROC=./05_launch_process.sh
STATE=./16_imu_state_publisher.sh
START_CONTAINER=./01_start_container.sh
START_CONTAINER_PROCESSOR=./A1_start_yolo_container.sh


# 1) start containers
"$START_CONTAINER"
"$START_CONTAINER_PROCESSOR"

# 2) acquisition (bg; logs handled in 06)
"$ACQ" start
"$ACQ2" start
"$ACQ3" start

# 3) image_saver with min_distance negative to bypass that check 
"$IMG" start --ros-args -p min_distance:=-0.5 -p enabled_topic:=/imu/is_moving
# 4) processor launch file
#PER ORA COMMENTATO:NIENTE PROCESSING
#"$PROC" start

# 3-4b) wait briefly so the first state msg isn’t missed
echo "⏳ waiting for image_saver…"; sleep 2

# 4) file state publisher
"$STATE" start

echo "🎯 Pipeline started. Use:"
echo "  $ACQ tail"
echo "  $ACQ2 tail"
echo "  $ACQ3 tail"
echo "  $IMG tail"
echo "  $STATE tail"
#echo "  $PROC tail"
