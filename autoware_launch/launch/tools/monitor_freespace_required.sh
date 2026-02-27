#!/usr/bin/env bash

set -u
set -o pipefail

LOG_FILE="${HOME}/autoware_freespace_required.log"
INTERVAL_SEC="${INTERVAL_SEC:-2}"
ECHO_TIMEOUT_SEC="${ECHO_TIMEOUT_SEC:-1.5}"
TF_TIMEOUT_SEC="${TF_TIMEOUT_SEC:-2}"
ROS2_CMD=(ros2)

TOPIC_INFOS=(
  "/initialpose3d"
  "/tf"
  "/localization/kinematic_state"
  "/planning/scenario_planning/scenario"
  "/planning/scenario_planning/parking/static_occupancy_grid"
  "/planning/scenario_planning/parking/trajectory"
  "/planning/trajectory"
  "/control/trajectory_follower/control_cmd"
  "/control/command/control_cmd"
  "/control/is_autonomous_available"
  "/api/operation_mode/state"
  "/system/operation_mode/state"
)

DYNAMIC_TOPICS=(
  "/initialpose3d"
  "/localization/kinematic_state"
  "/planning/scenario_planning/parking/trajectory"
  "/planning/trajectory"
  "/control/trajectory_follower/control_cmd"
  "/control/command/control_cmd"
)

timestamp() {
  date "+%Y-%m-%d %H:%M:%S"
}

log_line() {
  local line="$1"
  echo "[$(timestamp)] ${line}" | tee -a "${LOG_FILE}"
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

check_autonomous_available() {
  local raw
  raw="$(timeout "${ECHO_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" topic echo /control/is_autonomous_available --once 2>/dev/null || true)"
  if [[ -z "${raw}" ]]; then
    log_line "AUTO FLAG     /control/is_autonomous_available unavailable"
    return
  fi

  local data
  data="$(awk -F': ' '/data:/{print $2; exit}' <<< "${raw}")"
  log_line "AUTO FLAG     /control/is_autonomous_available data=${data:-unknown}"
}

check_operation_mode_state() {
  local raw
  raw="$(timeout "${ECHO_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" topic echo /api/operation_mode/state --once 2>/dev/null || true)"
  if [[ -z "${raw}" ]]; then
    log_line "MODE STATE    /api/operation_mode/state unavailable"
    return
  fi

  local mode auto_available autoware_control
  mode="$(awk -F': ' '/mode:/{print $2; exit}' <<< "${raw}")"
  auto_available="$(awk -F': ' '/is_autonomous_mode_available:/{print $2; exit}' <<< "${raw}")"
  autoware_control="$(awk -F': ' '/is_autoware_control_enabled:/{print $2; exit}' <<< "${raw}")"

  log_line "MODE STATE    mode=${mode:-unknown} auto_available=${auto_available:-unknown} autoware_control=${autoware_control:-unknown}"
}

check_parking_vs_main_trajectory() {
  local parking_ok main_ok

  if timeout "${ECHO_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" topic echo /planning/scenario_planning/parking/trajectory --once >/dev/null 2>&1; then
    parking_ok="yes"
  else
    parking_ok="no"
  fi

  if timeout "${ECHO_TIMEOUT_SEC}s" "${ROS2_CMD[@]}" topic echo /planning/trajectory --once >/dev/null 2>&1; then
    main_ok="yes"
  else
    main_ok="no"
  fi

  log_line "TRAJ STATUS   parking=${parking_ok} planning=${main_ok}"
  if [[ "${parking_ok}" == "yes" && "${main_ok}" == "no" ]]; then
    log_line "TRAJ WARNING  parking trajectory exists but /planning/trajectory missing"
  fi
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
  echo "===== freespace required monitor start $(timestamp) =====" >> "${LOG_FILE}"
  log_line "monitor started"
  log_line "fixed log file: ${LOG_FILE}"
  log_line "interval: ${INTERVAL_SEC}s"
  log_line "ECHO_TIMEOUT_SEC=${ECHO_TIMEOUT_SEC}"
  log_line "TF_TIMEOUT_SEC=${TF_TIMEOUT_SEC}"
  log_line "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  log_line "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"

  trap on_exit EXIT
  trap on_signal INT TERM

  while true; do
    log_line "----- tick -----"

    for topic in "${TOPIC_INFOS[@]}"; do
      check_topic_info "${topic}"
    done

    for topic in "${DYNAMIC_TOPICS[@]}"; do
      check_topic_message_once "${topic}"
    done

    check_parking_vs_main_trajectory
    check_tf_map_to_base_link
    check_autonomous_available
    check_operation_mode_state

    sleep "${INTERVAL_SEC}"
  done
}

main "$@"
