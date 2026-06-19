#!/bin/bash

set -e

MODE="shell"
if [[ "${1:-}" == "--web" ]]; then
    MODE="web"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# allow any application to connect to the X server
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker || true
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker command not found"
    exit 1
fi

DOCKER_PREFIX=()
if ! docker info >/dev/null 2>&1; then
    if command -v sudo >/dev/null 2>&1; then
        DOCKER_PREFIX=(sudo)
    else
        echo "Error: docker requires elevated permissions and sudo is not available"
        exit 1
    fi
fi

CONTAINER_NAME="aidf_container"
EXISTING_CONTAINER_ID=$("${DOCKER_PREFIX[@]}" docker ps -aq --filter "name=^/${CONTAINER_NAME}$")
if [[ -n "$EXISTING_CONTAINER_ID" ]]; then
    echo "Removing existing container ${CONTAINER_NAME} (${EXISTING_CONTAINER_ID})"
    "${DOCKER_PREFIX[@]}" docker rm -f "$CONTAINER_NAME" >/dev/null
fi

cleanup_container() {
    "${DOCKER_PREFIX[@]}" docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}

on_interrupt() {
    echo "Received interrupt. Stopping ${CONTAINER_NAME}..."
    cleanup_container
    exit 130
}

on_suspend() {
    echo "Received suspend signal. Stopping ${CONTAINER_NAME} instead of suspending."
    cleanup_container
    exit 148
}

trap on_interrupt INT TERM
trap on_suspend TSTP

# run docker with x11 forwarding
DOCKER_ARGS_BASE=(run -it \
    --network host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${REPO_ROOT}:/root/catkin_ws/src/AIDF:rw" \
    --name="$CONTAINER_NAME")

if [[ "$MODE" == "web" ]]; then
    DOCKER_ARGS=("${DOCKER_ARGS_BASE[@]}" --entrypoint /bin/bash aidf-image /root/catkin_ws/src/AIDF/docker/start_web_services.sh)
else
    DOCKER_ARGS=("${DOCKER_ARGS_BASE[@]}" aidf-image)
fi

"${DOCKER_PREFIX[@]}" docker "${DOCKER_ARGS[@]}"
