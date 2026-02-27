#!/usr/bin/env bash

set -u

LOG_FILE="${HOME}/autoware_flow_monitor.log"
INTERVAL_SEC=2
ECHO_TIMEOUT_SEC=2
TF_TIMEOUT_SEC=3
ROS2_CMD=(ros2)

NODES=(
  "/map/lanelet2_map_loader"
  "/planning/mission_planning/mission_planner"
  "/planning/scenario_planning/parking/costmap_generator"
  "/planning/scenario_planning/parking/freespace_planner"
  "/planning/planning_freespace_route_bridge"
  "/planning/scenario_planning/parking/parking_static_occupancy_map_publisher"
  "/planning/scenario_planning/velocity_smoother"
)

TOPICS=(
  "/tf"
  "/localization/kinematic_state"
  "/planning/mission_planning/route"
  "/planning/scenario_planning/scenario"
  "/planning/scenario_planning/parking/costmap_generator/occupancy_grid_raw"
  "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"
  "/planning/scenario_planning/parking/static_occupancy_grid"
  "/planning/scenario_planning/parking/trajectory"
  "/planning/trajectory"
  "/initialpose3d"
)

MESSAGE_TOPICS=(
  "/localization/kinematic_state"
  "/planning/mission_planning/route"
  "/planning/scenario_planning/scenario"
  "/planning/scenario_planning/parking/costmap_generator/occupancy_grid_raw"
  "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"
  "/planning/scenario_planning/parking/static_occupancy_grid"
)

timestamp() {
  date "+%Y-%m-%d %H:%M:%S"
}

log_line() {
  local line="$1"
  echo "[$(timestamp)] ${line}" | tee -a "${LOG_FILE}"
}

check_node_alive() {
  local nodes_text="$1"
  local node="$2"
  if grep -Fxq "${node}" <<< "${nodes_text}"; then
    log_line "NODE OK      ${node}"
  else
    log_line "NODE MISSING ${node}"
  fi
}

check_topic_info() {
  local topic="$1"
  local info
  info="$("${ROS2_CMD[@]}" topic info "${topic}" 2>/dev/null || true)"

  if [[ -z "${info}" ]]; then
    log_line "TOPIC MISSING ${topic}"
    return
  fi

  local type pubs subs
  type="$(awk '/Type:/{print $2}' <<< "${info}" | head -n1)"
  pubs="$(awk '/Publisher count:/{print $3}' <<< "${info}" | head -n1)"
  subs="$(awk '/Subscription count:/{print $3}' <<< "${info}" | head -n1)"

  log_line "TOPIC OK      ${topic} type=${type:-unknown} pub=${pubs:-0} sub=${subs:-0}"
}

check_topic_message_once() {
  local topic="$1"
  if timeout "${ECHO_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" topic echo "${topic}" --once >/dev/null 2>&1; then
    log_line "MSG OK        ${topic}"
  else
    log_line "MSG TIMEOUT   ${topic} (>${ECHO_TIMEOUT_SEC}s)"
  fi
}

check_tf_map_to_base_link() {
  local tmp
  tmp="$(mktemp)"
  timeout "${TF_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" run tf2_ros tf2_echo map base_link >"${tmp}" 2>&1 || true

  if grep -Eq "At time|Translation" "${tmp}"; then
    log_line "TF OK         map -> base_link"
  else
    local reason
    reason="$(head -n1 "${tmp}" | tr -d '\r')"
    if [[ -z "${reason}" ]]; then
      reason="no output"
    fi
    log_line "TF MISSING    map -> base_link (${reason})"
  fi

  rm -f "${tmp}"
}

on_exit() {
  log_line "monitor stopped"
}

on_signal() {
  exit 0
}

main() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found. Please source ROS 2 environment first." >&2
    exit 1
  fi

  touch "${LOG_FILE}"
  echo "" >> "${LOG_FILE}"
  echo "===== monitor session start $(timestamp) =====" >> "${LOG_FILE}"
  log_line "monitor started"
  log_line "fixed log file: ${LOG_FILE}"
  log_line "interval: ${INTERVAL_SEC}s"
  log_line "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  log_line "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"

  trap on_exit EXIT
  trap on_signal INT TERM

  while true; do
    log_line "----- tick -----"

    local nodes_text
    nodes_text="$("${ROS2_CMD[@]}" node list 2>/dev/null || true)"
    for node in "${NODES[@]}"; do
      check_node_alive "${nodes_text}" "${node}"
    done

    for topic in "${TOPICS[@]}"; do
      check_topic_info "${topic}"
    done

    for topic in "${MESSAGE_TOPICS[@]}"; do
      check_topic_message_once "${topic}"
    done

    check_tf_map_to_base_link
    sleep "${INTERVAL_SEC}"
  done
}

main "$@"
