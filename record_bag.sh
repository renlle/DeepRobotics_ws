#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${SCRIPT_DIR}/log/nav_turn_issue_${STAMP}"
ALGORITHM_OUTPUT="${OUTPUT_ROOT}/algorithm_domain_42"
CHASSIS_OUTPUT="${OUTPUT_ROOT}/chassis_domain_0"
STOPPED=0

mkdir -p "${OUTPUT_ROOT}"



ROS_DOMAIN_ID=42 ros2 bag record -o "${ALGORITHM_OUTPUT}" \
  /rosout \
  /runtime/performance \
  /global_path \
  /waypoint_global_path \
  /goal_pose \
  /clicked_point \
  /waypoints_pose_array \
  /waypoint_sequence/status \
  /waypoint_sequence/clear \
  /waypoint_sequence/pause \
  /waypoint_sequence/resume \
  /chassis/control_usage_mode \
  /local_path \
  /local_path_rl_debug \
  /mppi_path \
  /subgoal \
  /final_goal \
  /follow_path/_action/status \
  /global_localization/path \
  /localization \
  /odom \
  /aft_mapped_to_init \
  /livox/imu \
  /cmd_vel_nav \
  /cmd_vel \
  /NAV_CMD \
  /scan \
  /global_costmap/costmap \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /tf \
  /tf_static \
  > >(tee "${OUTPUT_ROOT}/algorithm_recorder.log") \
  2> >(tee "${OUTPUT_ROOT}/algorithm_recorder_error.log" >&2) &
ALGORITHM_PID=$!

ROS_DOMAIN_ID=0 ros2 bag record -o "${CHASSIS_OUTPUT}" \
  /rosout \
  /NAV_CMD \
  /CONTROL_USAGE_MODE_MASTER \
  /MOTION_STATE \
  > >(tee "${OUTPUT_ROOT}/chassis_recorder.log") \
  2> >(tee "${OUTPUT_ROOT}/chassis_recorder_error.log" >&2) &
CHASSIS_PID=$!

stop_recording() {
  if [[ "${STOPPED}" -eq 1 ]]; then
    return
  fi
  STOPPED=1
  printf '\n正在停止录包并写入元数据...\n'
  kill -INT "${ALGORITHM_PID}" "${CHASSIS_PID}" 2>/dev/null || true
  wait "${ALGORITHM_PID}" 2>/dev/null || true
  wait "${CHASSIS_PID}" 2>/dev/null || true
  printf '录包完成：%s\n' "${OUTPUT_ROOT}"
  for metadata in "${ALGORITHM_OUTPUT}/metadata.yaml" "${CHASSIS_OUTPUT}/metadata.yaml"; do
    if [[ ! -f "${metadata}" ]]; then
      printf '错误：缺少 %s，录包未正常收尾，请查看 recorder_error.log。\n' "${metadata}" >&2
    fi
  done
}

trap 'stop_recording; exit 130' INT TERM
trap 'stop_recording' EXIT

printf '同时记录算法域 Domain 42 和底盘域 Domain 0。\n'
printf '按 q 或 Ctrl+C 停止。输出目录：%s\n' "${OUTPUT_ROOT}"

if [[ -t 0 ]]; then
  while kill -0 "${ALGORITHM_PID}" 2>/dev/null && kill -0 "${CHASSIS_PID}" 2>/dev/null; do
    if IFS= read -rsn1 -t 0.2 key && [[ "${key}" == "q" || "${key}" == "Q" ]]; then
      break
    fi
  done
else
  while kill -0 "${ALGORITHM_PID}" 2>/dev/null && kill -0 "${CHASSIS_PID}" 2>/dev/null; do
    sleep 0.5
  done
fi

stop_recording
