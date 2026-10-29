#!/usr/bin/env bash
# 独立运行保持前台；start_app.sh 将其放到后台并保存日志。
set -eo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"
# 本机默认 DDS 传输可发现底盘却收不到数据；复用项目已验证的 UDP 配置。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${WORKSPACE_DIR}/config/fastdds_udp_only.xml}"
# 保留 Python 入口及锁身份，确保可替换更名前已启动的旧桥。
exec /usr/bin/python3 -u "${WORKSPACE_DIR}/scripts/battery_domain_bridge.py" "$@"
