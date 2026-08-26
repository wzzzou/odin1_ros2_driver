#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ODIN_WORKSPACE="${ODIN_WORKSPACE:-${HOME}/odin_ws}"
ODIN_LAUNCH_FILE="${ODIN_LAUNCH_FILE:-odin1_odom.launch.py}"
ODIN_ROS_DOMAIN_ID="${ODIN_ROS_DOMAIN_ID:-7}"
ODIN_REMOTE_MODE="${ODIN_REMOTE_MODE:-lite}"
ODIN_VOXEL_LEAF_SIZE="${ODIN_VOXEL_LEAF_SIZE:-0.40}"
ODIN_IMAGE_REMOTE_MAX_WIDTH="${ODIN_IMAGE_REMOTE_MAX_WIDTH:-480}"
ODIN_IMAGE_REMOTE_JPEG_QUALITY="${ODIN_IMAGE_REMOTE_JPEG_QUALITY:-45}"

case "${ODIN_REMOTE_MODE}" in
    lite)
        # 降级显示模式：适合 NUC 自身 WiFi 发布端或强干扰场景。
        # 保留完整 /odin1/cloud_slam，同时额外发布小包 /odin1/cloud_slam_lite。
        ODIN_ENABLE_CLOUD_RELAY="${ODIN_ENABLE_CLOUD_RELAY:-1}"
        ODIN_ENABLE_REMOTE_RELAY="${ODIN_ENABLE_REMOTE_RELAY:-1}"
        ODIN_ENABLE_REMOTE_IMAGE="${ODIN_ENABLE_REMOTE_IMAGE:-1}"
        ODIN_REMOTE_LIGHT_CONFIG="${ODIN_REMOTE_LIGHT_CONFIG:-1}"
        ;;
    full)
        # 完整点云直传模式：适合 NUC 有线接车载路由器、笔记本连路由器 WiFi。
        # 默认只保留完整点云/odom/path/TF，关闭降采样和图像中继，降低额外带宽。
        ODIN_ENABLE_CLOUD_RELAY="${ODIN_ENABLE_CLOUD_RELAY:-0}"
        ODIN_ENABLE_REMOTE_RELAY="${ODIN_ENABLE_REMOTE_RELAY:-0}"
        ODIN_ENABLE_REMOTE_IMAGE="${ODIN_ENABLE_REMOTE_IMAGE:-0}"
        ODIN_REMOTE_LIGHT_CONFIG="${ODIN_REMOTE_LIGHT_CONFIG:-1}"
        ;;
    *)
        echo "ODIN_REMOTE_MODE 只能是 lite 或 full，当前为：${ODIN_REMOTE_MODE}" >&2
        exit 1
        ;;
esac

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
    echo "找不到 ROS 环境：/opt/ros/${ROS_DISTRO_NAME}/setup.bash" >&2
    exit 1
fi

if [[ ! -f "${ODIN_WORKSPACE}/install/setup.bash" ]]; then
    echo "找不到 Odin 工作区安装环境：${ODIN_WORKSPACE}/install/setup.bash" >&2
    exit 1
fi

set +u  # ROS setup.bash 内部引用了未绑定变量，需临时关闭 -u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "${ODIN_WORKSPACE}"
source install/setup.bash
set -u

ODIN_PACKAGE_PREFIX="$(ros2 pkg prefix odin_ros_driver)"
ODIN_PACKAGE_SHARE="${ODIN_PACKAGE_PREFIX}/share/odin_ros_driver"
ODIN_PACKAGE_LIBEXEC="${ODIN_PACKAGE_PREFIX}/lib/odin_ros_driver"
ODIN_WORKSPACE_INSTALL="$(realpath -m "${ODIN_WORKSPACE}/install")"
ODIN_PACKAGE_PREFIX_REAL="$(realpath -m "${ODIN_PACKAGE_PREFIX}")"

case "${ODIN_PACKAGE_PREFIX_REAL}" in
    "${ODIN_WORKSPACE_INSTALL}"|"${ODIN_WORKSPACE_INSTALL}"/*) ;;
    *)
        echo "当前 overlay 不属于 ODIN_WORKSPACE，拒绝混用：${ODIN_PACKAGE_PREFIX_REAL}" >&2
        exit 1
        ;;
esac

if [[ ! -d "${ODIN_PACKAGE_SHARE}" || ! -d "${ODIN_PACKAGE_LIBEXEC}" ]]; then
    echo "当前 overlay 中的 odin_ros_driver 安装不完整：${ODIN_PACKAGE_PREFIX}" >&2
    exit 1
fi

if [[ "${ODIN_ENABLE_REMOTE_RELAY}" == "1" && ! -f "${ODIN_PACKAGE_SHARE}/script/odin_remote_topic_relay.py" ]]; then
    echo "找不到已安装的远程图像中继：${ODIN_PACKAGE_SHARE}/script/odin_remote_topic_relay.py" >&2
    exit 1
fi

if [[ "${ODIN_ENABLE_CLOUD_RELAY}" == "1" && ! -x "${ODIN_PACKAGE_LIBEXEC}/cloud_downsample_relay" ]]; then
    echo "找不到已安装的点云中继：${ODIN_PACKAGE_LIBEXEC}/cloud_downsample_relay" >&2
    exit 1
fi

export ROS_DOMAIN_ID="${ODIN_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ODIN_REMOTE_MODE=${ODIN_REMOTE_MODE}"
echo "ODIN_ENABLE_CLOUD_RELAY=${ODIN_ENABLE_CLOUD_RELAY}"
echo "ODIN_ENABLE_REMOTE_RELAY=${ODIN_ENABLE_REMOTE_RELAY}"
echo "ODIN_ENABLE_REMOTE_IMAGE=${ODIN_ENABLE_REMOTE_IMAGE}"
echo "启动 NUC Odin headless：${ODIN_LAUNCH_FILE}"

EXTRA_ARGS=("$@")
HAS_CONFIG_FILE_ARG=0
for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "${arg}" == config_file:=* ]]; then
        HAS_CONFIG_FILE_ARG=1
        break
    fi
done

if [[ "${ODIN_REMOTE_LIGHT_CONFIG}" == "1" && "${HAS_CONFIG_FILE_ARG}" == "0" && "${ODIN_LAUNCH_FILE}" == "odin1_odom.launch.py" ]]; then
    REMOTE_CONFIG="/tmp/odin1_odom_remote_light.yaml"
    BASE_CONFIG="${ODIN_PACKAGE_SHARE}/config/control_odom.yaml"
    cp "${BASE_CONFIG}" "${REMOTE_CONFIG}"
    sed -i 's/^  sendrgb:.*/  sendrgb: 0/' "${REMOTE_CONFIG}"
    if [[ "${ODIN_ENABLE_REMOTE_IMAGE}" == "1" ]]; then
        sed -i 's/^  sendrgbcompressed:.*/  sendrgbcompressed: 1/' "${REMOTE_CONFIG}"
    else
        sed -i 's/^  sendrgbcompressed:.*/  sendrgbcompressed: 0/' "${REMOTE_CONFIG}"
    fi
    sed -i 's/^  sendrgbundistort:.*/  sendrgbundistort: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  sendodomhighfreq:.*/  sendodomhighfreq: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  sendodomtfstream:.*/  sendodomtfstream: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  sendwiwc:.*/  sendwiwc: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  senddtof:.*/  senddtof: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  sendcloudrender:.*/  sendcloudrender: 0/' "${REMOTE_CONFIG}"
    sed -i 's/^  showpath:.*/  showpath: 1/' "${REMOTE_CONFIG}"
    sed -i 's/^  sdk_log_level:.*/  sdk_log_level: 0/' "${REMOTE_CONFIG}"
    EXTRA_ARGS=("config_file:=${REMOTE_CONFIG}" "${EXTRA_ARGS[@]}")
    echo "使用远程轻量配置：${REMOTE_CONFIG}"
fi

RELAY_PIDS=()
cleanup() {
    for relay_pid in "${RELAY_PIDS[@]}"; do
        if [[ -n "${relay_pid}" ]] && kill -0 "${relay_pid}" 2>/dev/null; then
            kill -TERM "${relay_pid}" 2>/dev/null || true
            wait "${relay_pid}" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

if [[ "${ODIN_ENABLE_CLOUD_RELAY}" == "1" ]]; then
    echo "启动点云降采样中继：/odin1/cloud_slam -> /odin1/cloud_slam_lite (voxel=${ODIN_VOXEL_LEAF_SIZE}m)"
    "${ODIN_PACKAGE_LIBEXEC}/cloud_downsample_relay" \
        --ros-args -p voxel_leaf_size:="${ODIN_VOXEL_LEAF_SIZE}" &
    RELAY_PIDS+=("$!")
fi

if [[ "${ODIN_ENABLE_REMOTE_RELAY}" == "1" ]]; then
    REMOTE_IMAGE_PARAM="false"
    if [[ "${ODIN_ENABLE_REMOTE_IMAGE}" == "1" || "${ODIN_ENABLE_REMOTE_IMAGE}" == "true" ]]; then
        REMOTE_IMAGE_PARAM="true"
    fi
    echo "启动远程图像中继：/odin1/image/compressed -> /odin1/image_remote/compressed (image=${ODIN_ENABLE_REMOTE_IMAGE})"
    python3 "${ODIN_PACKAGE_SHARE}/script/odin_remote_topic_relay.py" \
        --ros-args \
        -p enable_pose:=false \
        -p enable_image:="${REMOTE_IMAGE_PARAM}" \
        -p image_max_width:="${ODIN_IMAGE_REMOTE_MAX_WIDTH}" \
        -p jpeg_quality:="${ODIN_IMAGE_REMOTE_JPEG_QUALITY}" &
    RELAY_PIDS+=("$!")
fi

ros2 launch odin_ros_driver "${ODIN_LAUNCH_FILE}" launch_rviz:=false "${EXTRA_ARGS[@]}"
