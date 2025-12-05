#!/bin/bash
# Runs FFmpeg integration tests inside an ephemeral Docker container.
#   precondition: either variable IMAGE_HEADLESS or IMAGE_HEADFUL is set to an image name
#       e.g. IMAGE_HEADLESS=onkernel/chromium-headless-test:latest ./test-ffmpeg-integration.sh

set -euo pipefail

CONTAINER_NAME="ffmpeg-integration-test"
IMAGE_HEADLESS="${IMAGE_HEADLESS:-}"
IMAGE_HEADFUL="${IMAGE_HEADFUL:-}"
if [[ -n "$IMAGE_HEADLESS" && -n "$IMAGE_HEADFUL" ]]; then
    echo "Either IMAGE_HEADLESS or IMAGE_HEADFUL should be set, not both."
    exit 1
elif [[ -n "$IMAGE_HEADLESS" ]]; then
    IMAGE="$IMAGE_HEADLESS"
    CONTAINER_NAME+="-headless"
    RUN_DOCKER_SCRIPT_PATH="../images/chromium-headless/run-docker.sh"
elif [[ -n "$IMAGE_HEADFUL" ]]; then
    IMAGE="$IMAGE_HEADFUL"
    CONTAINER_NAME+="-headful"
    RUN_DOCKER_SCRIPT_PATH="../images/chromium-headful/run-docker.sh"
else
    echo "Usage: Either IMAGE_HEADFUL or IMAGE_HEADLESS must be set to an image name."
    echo "    e.g.: IMAGE_HEADLESS=onkernel/chromium-headless-test:latest ./test-ffmpeg-integration.sh"
    exit 1
fi
CONTAINER_NAME+="-$$"

WIDTH=${WIDTH:-1920}
HEIGHT=${HEIGHT:-1080}
DISPLAY_NUM=${DISPLAY_NUM:-1}
TEST_TIMEOUT="${TEST_TIMEOUT:-30m}"
VERBOSE="${VERBOSE:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_BINARY_NAME="ffmpeg_integration_tests.test"
TEST_BINARY_PATH=$(mktemp -d)/${TEST_BINARY_NAME}
CONTAINER_STARTED=0

cleanup() {
    if [[ "$CONTAINER_STARTED" = "1" ]]; then
        echo "Stopping and removing container ..."
        docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
        docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
    fi

    if [[ -f "$TEST_BINARY_PATH" ]]; then
        echo "Cleaning up test binary ..."
        rm -rf $(dirname $TEST_BINARY_PATH)
    fi
}

trap cleanup EXIT

if ! docker image inspect "$IMAGE" &> /dev/null; then
    echo "Docker image '$IMAGE' not found."
    exit 1
fi

echo "Using Docker image: $IMAGE"

echo "Compiling integration tests ..."
pushd "$SCRIPT_DIR"
if ! go test -tags=integration -c -o "$TEST_BINARY_PATH"; then
    echo "Failed to compile integration tests"
    exit 1
fi
popd
echo "Test binary compiled: $TEST_BINARY_PATH"

TEST_FLAGS="-test.timeout=$TEST_TIMEOUT"
if [[ "$VERBOSE" = "1" ]]; then
    TEST_FLAGS="$TEST_FLAGS -test.v"
fi

echo "Starting container $CONTAINER_NAME based on $IMAGE in background ..."
if ! DOCKER_DETACH=true DOCKER_EPHEMERAL=true NAME="$CONTAINER_NAME" IMAGE="$IMAGE" $RUN_DOCKER_SCRIPT_PATH; then
    echo "Failed to start container. Exiting."
    exit 1
fi
CONTAINER_STARTED=1
echo "Started container."

WAIT_SECONDS=0
WAIT_MAX_SECONDS=30
echo "Waiting up to ${WAIT_MAX_SECONDS}s for X server to be ready ..."

while [[ $WAIT_SECONDS -lt $WAIT_MAX_SECONDS ]]; do
    if ! docker exec --env DISPLAY=:$DISPLAY_NUM "$CONTAINER_NAME" bash -c "xrandr -q >/dev/null 2>&1"; then
        printf "\r  Container isn't ready yet. Waiting ... ($WAIT_SECONDS / $WAIT_MAX_SECONDS seconds)"
    else
        printf "\nContainer X server is ready.\n"
        break
    fi

    sleep 1
    WAIT_SECONDS=$((WAIT_SECONDS + 1))
done

if [[ $WAIT_SECONDS -ge $WAIT_MAX_SECONDS ]]; then
    echo "Container with X server did not become ready within $WAIT_MAX_SECONDS seconds."
    docker logs "$CONTAINER_NAME" 2>&1 | tail -20
    exit 1
fi

echo "FFmpeg version in container:"
docker exec "$CONTAINER_NAME" ffmpeg -version 2>/dev/null | head -n 1

echo "Copying test binary into container ..."
if ! docker cp "$TEST_BINARY_PATH" "$CONTAINER_NAME:/tmp/$TEST_BINARY_NAME"; then
    echo "Failed to copy test binary into container"
    exit 1
fi

echo "Copying testdata directory into container ..."
if ! docker cp lib/recorder/testdata/ "$CONTAINER_NAME:/tmp/testdata"; then
    echo "Failed to copy testdata directory into container"
    exit 1
fi

echo "Setting X display resolution to $WIDTH x $HEIGHT ..."
if ! docker exec --env DISPLAY=:$DISPLAY_NUM "$CONTAINER_NAME" bash -c "xrandr -s ${WIDTH}x${HEIGHT}"; then
    echo "Failed to set initial display resolution."
    docker logs "$CONTAINER_NAME" 2>&1 | tail -20
    exit 1
fi

echo "Running tests as kernel user ..."
if docker exec --workdir="/tmp" --user kernel --env DISPLAY=:$DISPLAY_NUM "$CONTAINER_NAME" ./$TEST_BINARY_NAME $TEST_FLAGS; then
    echo "All integration tests passed!"
    exit 0
else
    EXIT_CODE=$?
    echo "Integration tests failed with exit code $EXIT_CODE"
    exit $EXIT_CODE
fi
