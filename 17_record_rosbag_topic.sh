#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Carica config.sh se esiste
if [[ -f "${SCRIPT_DIR}/config.sh" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/config.sh"
fi

: "${CONTAINER_NAME:=owl-dev}"
: "${BAG_ROOT:=/home/dev/bags}"
: "${BAG_PREFIX:=owl}"
: "${BAG_STORAGE:=sqlite3}"
: "${BAG_EXTRA_ARGS:=}"
: "${EXCLUDE_PATTERN:=camera}"
: "${EXCLUDE_ROSOUT:=1}"

PID_FILE="${BAG_ROOT}/.rosbag_record.pid"
PGID_FILE="${BAG_ROOT}/.rosbag_record.pgid"
INFO_FILE="${BAG_ROOT}/.rosbag_record.info"
LOG_FILE="${BAG_ROOT}/.rosbag_record.log"

usage() {
  cat <<EOF
Uso:
  $0 start [topic1 topic2 ...]
  $0 stop
  $0 status
  $0 tail

Default topics:
  ros2 topic list | grep -v '${EXCLUDE_PATTERN}'  (e senza /rosout se EXCLUDE_ROSOUT=1)

Salva in:
  ${BAG_ROOT}/DD-MM-YYYY/<bagname>

Override:
  TOPICS="/tf /tf_static /imu/data" $0 start
  $0 start /tf /tf_static

Env:
  EXCLUDE_PATTERN='camera|image'
  BAG_STORAGE=sqlite3|mcap
  BAG_EXTRA_ARGS="--compression-mode file --compression-format zstd"
EOF
}

normalize_ws() { tr -d '\r' | tr '\n' ' ' | awk '{$1=$1; print}'; }

docker_flags_interactive() {
  if [[ -t 0 && -t 1 ]]; then echo "-it"; else echo "-i"; fi
}

docker_exec() {
  local flags; flags="$(docker_flags_interactive)"
  docker exec ${flags} "${CONTAINER_NAME}" bash -lc "$*"
}

docker_exec_bg() {
  docker exec "${CONTAINER_NAME}" bash -lc "$*"
}

container_check() {
  if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "[ERRORE] Container '${CONTAINER_NAME}' non trovato o non running."
    docker ps --format '  - {{.Names}}'
    exit 1
  fi
}

is_running() {
  local pid
  pid="$(docker_exec "cat '${PID_FILE}' 2>/dev/null || true" | normalize_ws || true)"
  [[ -n "${pid}" ]] || return 1
  docker_exec "kill -0 '${pid}' 2>/dev/null" >/dev/null 2>&1
}

get_default_topics() {
  # IMPORTANT: pipeline TUTTA su una riga (niente '|' che inizia una nuova riga)
  local cmd
  if [[ "${EXCLUDE_ROSOUT}" == "1" ]]; then
    cmd="set -eo pipefail;
         export AMENT_TRACE_SETUP_FILES=\"\${AMENT_TRACE_SETUP_FILES:-}\";
         source /opt/ros/\${ROS_DISTRO}/setup.bash >/dev/null 2>&1 || true;
         [[ -f /home/dev/ws/install/setup.bash ]] && source /home/dev/ws/install/setup.bash >/dev/null 2>&1 || true;
         ros2 topic list | grep -v '${EXCLUDE_PATTERN}' | grep -v '^/rosout$'"
  else
    cmd="set -eo pipefail;
         export AMENT_TRACE_SETUP_FILES=\"\${AMENT_TRACE_SETUP_FILES:-}\";
         source /opt/ros/\${ROS_DISTRO}/setup.bash >/dev/null 2>&1 || true;
         [[ -f /home/dev/ws/install/setup.bash ]] && source /home/dev/ws/install/setup.bash >/dev/null 2>&1 || true;
         ros2 topic list | grep -v '${EXCLUDE_PATTERN}'"
  fi

  docker_exec "${cmd}" | normalize_ws
}

start_recording() {
  container_check

  if is_running; then
    echo "[WARN] rosbag record già attivo:"
    docker_exec "cat '${INFO_FILE}' 2>/dev/null || true" || true
    exit 0
  fi

  echo "[INFO] preparo cartelle..."
  docker_exec "mkdir -p '${BAG_ROOT}'"

  local day_dir bag_day_root
  day_dir="$(date +%d-%m-%Y)"
  bag_day_root="${BAG_ROOT}/${day_dir}"
  docker_exec "mkdir -p '${bag_day_root}'"

  local ts bag_name bag_path
  ts="$(date +%Y%m%d_%H%M%S)"
  bag_name="${BAG_PREFIX}_${ts}"
  bag_path="${bag_day_root}/${bag_name}"

  local topics=""
  if [[ $# -gt 0 ]]; then
    topics="$*"
  elif [[ -n "${TOPICS:-}" ]]; then
    topics="${TOPICS}"
  else
    topics="$(get_default_topics)"
  fi
  topics="$(printf '%s' "${topics}" | normalize_ws || true)"

  if [[ -z "${topics}" ]]; then
    echo "[ERRORE] lista topic vuota (EXCLUDE_PATTERN='${EXCLUDE_PATTERN}')."
    exit 1
  fi

  echo "== ROS2 BAG RECORD =="
  echo "Container   : ${CONTAINER_NAME}"
  echo "Day folder  : ${bag_day_root}"
  echo "Bag name    : ${bag_name}"
  echo "Storage     : ${BAG_STORAGE}"
  echo "Exclude     : ${EXCLUDE_PATTERN} (EXCLUDE_ROSOUT=${EXCLUDE_ROSOUT})"
  echo "Extra args  : ${BAG_EXTRA_ARGS}"
  echo "Topics      : ${topics}"
  echo

  docker_exec_bg "
    set -eo pipefail
    export AMENT_TRACE_SETUP_FILES=\"\${AMENT_TRACE_SETUP_FILES:-}\"
    source /opt/ros/\${ROS_DISTRO}/setup.bash
    [[ -f /home/dev/ws/install/setup.bash ]] && source /home/dev/ws/install/setup.bash

    mkdir -p '${bag_day_root}'
    : > '${LOG_FILE}'

    {
      echo 'day_folder=${bag_day_root}'
      echo 'bag_name=${bag_name}'
      echo 'bag_path=${bag_path}'
      echo 'storage=${BAG_STORAGE}'
      echo 'exclude_pattern=${EXCLUDE_PATTERN}'
      echo 'topics=${topics}'
    } > '${INFO_FILE}'

    setsid bash -lc \"
      exec ros2 bag record -o '${bag_path}' --storage '${BAG_STORAGE}' ${BAG_EXTRA_ARGS} ${topics}
    \" >> '${LOG_FILE}' 2>&1 &

    echo \$! > '${PID_FILE}'
    echo \$! > '${PGID_FILE}'
  "

  echo "[OK] start: avviato."
  echo "Bag: ${bag_path}"
}

stop_recording() {
  container_check

  local pgid
  pgid="$(docker_exec "cat '${PGID_FILE}' 2>/dev/null || true" | normalize_ws || true)"
  if [[ -z "${pgid}" ]]; then
    pgid="$(docker_exec "cat '${PID_FILE}' 2>/dev/null || true" | normalize_ws || true)"
  fi

  if [[ -z "${pgid}" ]]; then
    echo "[INFO] stop: nessun pid/pgid trovato."
    exit 0
  fi

  echo "[INFO] stop: SIGINT al process group (-${pgid})..."
  docker_exec "kill -INT -- -'${pgid}' 2>/dev/null || true" >/dev/null 2>&1 || true

  docker_exec "
    for i in {1..40}; do
      if kill -0 '${pgid}' 2>/dev/null; then sleep 0.2; else exit 0; fi
    done
    exit 1
  " >/dev/null 2>&1 && {
    docker_exec "rm -f '${PID_FILE}' '${PGID_FILE}'" >/dev/null 2>&1 || true
    echo "[OK] stop: fatto."
    exit 0
  }

  echo "[WARN] ancora vivo: SIGTERM..."
  docker_exec "kill -TERM -- -'${pgid}' 2>/dev/null || true" >/dev/null 2>&1 || true

  docker_exec "
    for i in {1..20}; do
      if kill -0 '${pgid}' 2>/dev/null; then sleep 0.2; else exit 0; fi
    done
    exit 1
  " >/dev/null 2>&1 && {
    docker_exec "rm -f '${PID_FILE}' '${PGID_FILE}'" >/dev/null 2>&1 || true
    echo "[OK] stop: fatto."
    exit 0
  }

  echo "[WARN] ancora vivo: SIGKILL (ultimo resort)..."
  docker_exec "kill -KILL -- -'${pgid}' 2>/dev/null || true" >/dev/null 2>&1 || true
  docker_exec "rm -f '${PID_FILE}' '${PGID_FILE}'" >/dev/null 2>&1 || true
  echo "[OK] stop: kill forzato."
}

status_recording() {
  container_check

  if is_running; then
    echo "[OK] rosbag record ATTIVO"
    docker_exec "cat '${INFO_FILE}' 2>/dev/null || true" || true
    echo -n "pid="; docker_exec "cat '${PID_FILE}' 2>/dev/null || true" | normalize_ws || true
    echo -n "pgid="; docker_exec "cat '${PGID_FILE}' 2>/dev/null || true" | normalize_ws || true
  else
    echo "[INFO] rosbag record NON attivo"
    docker_exec "cat '${INFO_FILE}' 2>/dev/null || true" || true
  fi
}

tail_log() {
  container_check
  echo "[INFO] tail -f ${LOG_FILE}"
  docker_exec "tail -n 200 -f '${LOG_FILE}'"
}

main() {
  local cmd="${1:-}"
  shift || true
  case "${cmd}" in
    start)  start_recording "$@" ;;
    stop)   stop_recording ;;
    status) status_recording ;;
    tail)   tail_log ;;
    ""|-h|--help|help) usage ;;
    *) echo "[ERRORE] comando sconosciuto: ${cmd}"; usage; exit 2 ;;
  esac
}

main "$@"
