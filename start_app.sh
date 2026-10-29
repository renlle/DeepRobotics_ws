#!/usr/bin/env bash
# ROS 2 环境脚本可能读取未定义变量，因此不启用 set -u。
set -eo pipefail

export ROS_DOMAIN_ID=42

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_CONTROL_PID=""

# 3090 容器使用系统 Python 和 ubuntu 用户已安装的依赖，无需 Conda。
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"

cd "${WORKSPACE_DIR}"

# 可在工作区根目录 .env 配置 ENV_NAME_SZS 设备参数后缀（不纳入 Git）。
# 与 app.py 使用相同解析规则，打印实际生效后缀；已导出的环境变量优先。
ENV_NAME_SZS="$(python3 -c 'import os, sys; from dotenv import load_dotenv; load_dotenv(sys.argv[1]); print(os.environ.get("ENV_NAME_SZS", ""))' "${WORKSPACE_DIR}/.env")"
export ENV_NAME_SZS
echo '$ENV_NAME_SZS': "$ENV_NAME_SZS"

# 直接持有节点 PID，退出时等待回收，避免只结束 ros2 launch 后留下孤立节点。
cleanup() {
    if [[ -n "${MASTER_CONTROL_PID}" ]] && kill -0 "${MASTER_CONTROL_PID}" 2>/dev/null; then
        kill "${MASTER_CONTROL_PID}" 2>/dev/null || true
        for ((i=0; i<50; i++)); do
            kill -0 "${MASTER_CONTROL_PID}" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "${MASTER_CONTROL_PID}" 2>/dev/null; then
            kill -KILL "${MASTER_CONTROL_PID}" 2>/dev/null || true
        fi
        wait "${MASTER_CONTROL_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

MASTER_CONTROL_EXE="$(ros2 pkg prefix master_control)/lib/master_control/navigation_command_node"
[[ -x "${MASTER_CONTROL_EXE}" ]] || { echo "总控入口不存在或不可执行：${MASTER_CONTROL_EXE}" >&2; exit 1; }
# 串行化旧进程清理和新节点启动，避免两个启动脚本同时创建总控。
mkdir -p "${WORKSPACE_DIR}/tmp"
exec 9>"${WORKSPACE_DIR}/tmp/start_app_master_control.lock"
flock 9
python3 "${WORKSPACE_DIR}/scripts/stop_navigation_command.py" \
    "${WORKSPACE_DIR}" "${ROS_DOMAIN_ID}" "${MASTER_CONTROL_EXE}"
# 与 navigation_command.launch.py 的默认节点名和参数保持一致。
"${MASTER_CONTROL_EXE}" --ros-args -r __node:=navigation_command_node \
    -p use_sim_time:=false -p require_navigation_subscriber:=true 9>&- &
MASTER_CONTROL_PID=$!
sleep 0.5
kill -0 "${MASTER_CONTROL_PID}" 2>/dev/null || { echo '新总控启动失败。' >&2; exit 1; }
flock -u 9
exec 9>&-
echo "已重新启动总控导航节点（PID=${MASTER_CONTROL_PID}）。"

# 底盘状态桥独立驻留；启动时替换本工作区同域对旧实例，转发电池和大写/ODOM。
mkdir -p "${WORKSPACE_DIR}/log"
nohup bash "${WORKSPACE_DIR}/scripts/start_domain0to42_bridge.sh" > "${WORKSPACE_DIR}/log/domain0to42_bridge.log" 2>&1 &

python3 -u robot_console/robot_console/app.py
