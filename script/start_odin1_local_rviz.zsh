#!/usr/bin/env zsh
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ODIN_WORKSPACE="${ODIN_WORKSPACE:-/home/wzoun/E_Drive/Linux_Data/Workspace/ros2/odin_ws}"
ODIN_REMOTE_MODE="${ODIN_REMOTE_MODE:-lite}"
ODIN_ROS_DOMAIN_ID="${ODIN_ROS_DOMAIN_ID:-7}"
ODIN_IMAGE_COMPRESSED_TOPIC="${ODIN_IMAGE_COMPRESSED_TOPIC:-/odin1/image_remote/compressed}"
ODIN_IMAGE_REMOTE_TOPIC="${ODIN_IMAGE_REMOTE_TOPIC:-/odin1/image_remote}"

case "${ODIN_REMOTE_MODE}" in
    lite)
        # 降级显示模式：订阅 cloud_slam_lite 和压缩后的远程图像。
        DEFAULT_RVIZ_FILE="odin_odom_remote.rviz"
        DEFAULT_IMAGE_REPUBLISH=1
        ;;
    full)
        # 完整点云直传模式：订阅原始 cloud_slam，不默认订阅图像，避免额外带宽。
        DEFAULT_RVIZ_FILE="odin_odom_remote_full.rviz"
        DEFAULT_IMAGE_REPUBLISH=0
        ;;
    *)
        print -u2 "ODIN_REMOTE_MODE 只能是 lite 或 full，当前为：${ODIN_REMOTE_MODE}"
        exit 1
        ;;
esac

ODIN_ENABLE_IMAGE_REPUBLISH="${ODIN_ENABLE_IMAGE_REPUBLISH:-${DEFAULT_IMAGE_REPUBLISH}}"

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.zsh" ]]; then
    print -u2 "找不到 ROS 环境：/opt/ros/${ROS_DISTRO_NAME}/setup.zsh"
    exit 1
fi

if [[ ! -f "${ODIN_WORKSPACE}/install/setup.zsh" ]]; then
    print -u2 "找不到 Odin 工作区安装环境：${ODIN_WORKSPACE}/install/setup.zsh"
    exit 1
fi

set +u  # ROS setup.zsh may read optional unset variables.
source "/opt/ros/${ROS_DISTRO_NAME}/setup.zsh"
source "${ODIN_WORKSPACE}/install/setup.zsh"
set -u

ODIN_PACKAGE_PREFIX="$(ros2 pkg prefix odin_ros_driver)"
ODIN_PACKAGE_SHARE="${ODIN_PACKAGE_PREFIX}/share/odin_ros_driver"
ODIN_WORKSPACE_INSTALL="$(realpath -m "${ODIN_WORKSPACE}/install")"
ODIN_PACKAGE_PREFIX_REAL="$(realpath -m "${ODIN_PACKAGE_PREFIX}")"

case "${ODIN_PACKAGE_PREFIX_REAL}" in
    "${ODIN_WORKSPACE_INSTALL}"|"${ODIN_WORKSPACE_INSTALL}"/*) ;;
    *)
        print -u2 "当前 overlay 不属于 ODIN_WORKSPACE，拒绝混用：${ODIN_PACKAGE_PREFIX_REAL}"
        exit 1
        ;;
esac

ODIN_RVIZ_CONFIG="${ODIN_RVIZ_CONFIG:-${ODIN_PACKAGE_SHARE}/config/${DEFAULT_RVIZ_FILE}}"

if [[ ! -f "${ODIN_RVIZ_CONFIG}" ]]; then
    print -u2 "找不到 RViz 配置：${ODIN_RVIZ_CONFIG}"
    exit 1
fi
if [[ "${ODIN_ENABLE_IMAGE_REPUBLISH}" == "1" && ! -f "${ODIN_PACKAGE_SHARE}/script/odin_compressed_image_relay.py" ]]; then
    print -u2 "找不到已安装的图像中继：${ODIN_PACKAGE_SHARE}/script/odin_compressed_image_relay.py"
    exit 1
fi

export ROS_DOMAIN_ID="${ODIN_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

print "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
print "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
print "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
print "ODIN_REMOTE_MODE=${ODIN_REMOTE_MODE}"
print "ODIN_ENABLE_IMAGE_REPUBLISH=${ODIN_ENABLE_IMAGE_REPUBLISH}"

IMAGE_REPUBLISH_PID=""
cleanup() {
    if [[ -n "${IMAGE_REPUBLISH_PID}" ]] && kill -0 "${IMAGE_REPUBLISH_PID}" 2>/dev/null; then
        kill -TERM "${IMAGE_REPUBLISH_PID}" 2>/dev/null || true
        wait "${IMAGE_REPUBLISH_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [[ "${ODIN_ENABLE_IMAGE_REPUBLISH}" == "1" ]]; then
    print "启动本机压缩图像解码：${ODIN_IMAGE_COMPRESSED_TOPIC} -> ${ODIN_IMAGE_REMOTE_TOPIC}"
    python3 "${ODIN_PACKAGE_SHARE}/script/odin_compressed_image_relay.py" \
        --ros-args \
        -p input_topic:="${ODIN_IMAGE_COMPRESSED_TOPIC}" \
        -p output_topic:="${ODIN_IMAGE_REMOTE_TOPIC}" &
    IMAGE_REPUBLISH_PID=$!
fi

print "启动本机 RViz：${ODIN_RVIZ_CONFIG}"

rviz2 -d "${ODIN_RVIZ_CONFIG}" "$@"
