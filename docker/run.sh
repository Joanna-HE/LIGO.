#!/bin/bash
# Run LIGO in Docker with RViz (X11) and host networking.
#
# Usage:
#   docker/run.sh                                     # roslaunch ligo mapping_avia.launch
#   docker/run.sh roslaunch ligo mapping_hesai32.launch
#   DATA_DIR=~/bags docker/run.sh bash                # interactive shell, bags mounted at /data
#
# Results (Log/, PCD/) are written to ./output on the host, owned by the calling user.
set -e

IMAGE="${IMAGE:-ligo:noetic}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_DIR}/output}"
DATA_DIR="${DATA_DIR:-$(pwd)}"
PKG_DIR=/catkin_ws/src/ligo

mkdir -p "${OUTPUT_DIR}/Log" "${OUTPUT_DIR}/PCD"
xhost +local: > /dev/null 2>&1 || true

# Run as the host user so files in ./output are not owned by root.
USER_ARGS=(--user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_HOME=/tmp/.ros)
if [ -n "${XAUTHORITY}" ] && [ -f "${XAUTHORITY}" ]; then
    USER_ARGS+=(-e XAUTHORITY=/tmp/.Xauthority -v "${XAUTHORITY}:/tmp/.Xauthority:ro")
fi

GPU_ARGS=()
if [ -e /dev/dri ]; then GPU_ARGS+=(--device /dev/dri); fi

TTY_ARGS=(-i)
if [ -t 0 ]; then TTY_ARGS=(-it); fi

exec docker run --rm "${TTY_ARGS[@]}" \
    --net=host \
    --ipc=host \
    "${USER_ARGS[@]}" \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${GPU_ARGS[@]}" \
    -v "${OUTPUT_DIR}/Log:${PKG_DIR}/Log" \
    -v "${OUTPUT_DIR}/PCD:${PKG_DIR}/PCD" \
    -v "${DATA_DIR}:/data:ro" \
    "${IMAGE}" "$@"
