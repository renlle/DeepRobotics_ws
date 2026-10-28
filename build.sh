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

# 仅在构建子进程中排除 Conda 搜索路径，避免系统编译器链接到 gait 的 ICU/C++ 库。
for variable in PATH LD_LIBRARY_PATH LIBRARY_PATH CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH; do
    IFS=: read -r -a entries <<< "${!variable:-}"
    filtered=()
    for entry in "${entries[@]}"; do
        case "${entry}" in
            *miniforge*|*miniconda*|*anaconda*) continue ;;
        esac
        if [[ -n "${CONDA_PREFIX:-}" && "${entry}" == "${CONDA_PREFIX}"* ]]; then
            continue
        fi
        [[ -z "${entry}" ]] || filtered+=("${entry}")
    done
    printf -v "${variable}" '%s' "$(IFS=:; echo "${filtered[*]}")"
    export "${variable}"
done
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE PYTHONHOME
source "${ROS_SETUP}"

# 基础接口与总控包必须随全工程编译；保留数组以明确构建范围要求，不使用
# --packages-up-to，避免意外排除工作区内其他算法包。
CORE_PACKAGES=(drdds master_control_interfaces master_control)
echo "全量编译工作区（包含：${CORE_PACKAGES[*]}）"

# 保留 install 和编译对象，仅刷新 CMake 缓存以清除旧 Conda 库、头文件和解释器路径。
# ROS 接口生成使用系统 Python，不影响运行时 gait 环境。
/usr/bin/python3 -m colcon build --symlink-install --cmake-clean-cache \
    --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    --no-warn-unused-cli

echo
echo "编译完成。请在新终端中执行："
echo "source \"${WORKSPACE_DIR}/install/setup.bash\""
