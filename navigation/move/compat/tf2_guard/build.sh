#!/usr/bin/env bash
set -eo pipefail
task_source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
task_workspace="$(cd "$task_source_dir/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
cmake -S "$task_source_dir" -B "$task_workspace/build/nav_tf2_guard" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$task_workspace/build/nav_tf2_guard" -j2
# 测试仅使用内存 TF Buffer，不初始化 ROS、不连接底盘。
LD_PRELOAD="$task_workspace/build/nav_tf2_guard/libnav_tf2_guard.so${LD_PRELOAD:+:$LD_PRELOAD}" \
  timeout -k 2 20 "$task_workspace/build/nav_tf2_guard/test_tf2_guard"
