#!/usr/bin/env bash
# ROS 2 setup.bash 会读取若干可能未定义的环境变量，因此不能启用 nounset（set -u）。
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "错误：未找到 ROS 2 环境脚本：${ROS_SETUP}" >&2
    exit 1
fi

cd "${WORKSPACE_DIR}"
source "${ROS_SETUP}"

# 默认增量编译，不删除 install，避免控制台运行时功能路径失效。
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    --no-warn-unused-cli

echo
echo "编译完成。请在新终端中执行："
echo "source \"${WORKSPACE_DIR}/install/setup.bash\""
