#!/usr/bin/env bash

set -euo pipefail

# ============================================================
# Settings
# ============================================================

CAMERA_WS="${HOME}/camera_ws"

DEPTHAI_SRC="${CAMERA_WS}/deps/depthai-core"
DEPTHAI_INSTALL="${CAMERA_WS}/depthai_install"

DEPTHAI_REPO="https://github.com/luxonis/depthai-core.git"

# 引数でbranch/tag/commitを指定可能
# 例:
# ./setup_depthai_cpp.sh <commit hash>
#
# 未指定の場合はmain
DEPTHAI_REF="ac928141ef7750bf3082b5e1c319a115618660e7" 


echo "========================================"
echo " DepthAI C++ setup"
echo "========================================"
echo "Source  : ${DEPTHAI_SRC}"
echo "Install : ${DEPTHAI_INSTALL}"
echo "Ref     : ${DEPTHAI_REF}"
echo


# ============================================================
# Ubuntu dependencies
# ============================================================

echo "[1/6] Installing dependencies..."

sudo apt update

sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libudev-dev \
    libusb-1.0-0-dev \
    libopencv-dev


# ============================================================
# Clone depthai-core
# ============================================================

echo "[2/6] Preparing depthai-core..."

mkdir -p "${CAMERA_WS}/deps"

if [ ! -d "${DEPTHAI_SRC}/.git" ]; then

    git clone \
        --recursive \
        "${DEPTHAI_REPO}" \
        "${DEPTHAI_SRC}"

else

    echo "depthai-core already exists."

    # ローカル変更がある場合は勝手に壊さない
    if ! git -C "${DEPTHAI_SRC}" diff --quiet ||
       ! git -C "${DEPTHAI_SRC}" diff --cached --quiet; then

        echo "ERROR: depthai-core has local changes."
        echo "Commit or discard them before running this script."
        exit 1
    fi

    git -C "${DEPTHAI_SRC}" fetch --tags origin
fi


# ============================================================
# Checkout version
# ============================================================

echo "[3/6] Checking out ${DEPTHAI_REF}..."

if [ "${DEPTHAI_REF}" = "main" ]; then

    git -C "${DEPTHAI_SRC}" checkout main
    git -C "${DEPTHAI_SRC}" pull --ff-only origin main

else

    git -C "${DEPTHAI_SRC}" checkout "${DEPTHAI_REF}"

fi

git -C "${DEPTHAI_SRC}" \
    submodule update \
    --init \
    --recursive


# ============================================================
# Configure
# ============================================================

echo "[4/6] Configuring..."

rm -rf "${DEPTHAI_SRC}/build"

cmake \
    -S "${DEPTHAI_SRC}" \
    -B "${DEPTHAI_SRC}/build" \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${DEPTHAI_INSTALL}"


# ============================================================
# Build + install
# ============================================================

echo "[5/6] Building depthai-core..."

cmake \
    --build "${DEPTHAI_SRC}/build" \
    --target install \
    --parallel 1


# ============================================================
# OAK-D USB permission
# ============================================================

echo "[6/6] Installing OAK-D udev rule..."

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/80-movidius.rules \
    > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger


# ============================================================
# Verify
# ============================================================

DEPTHAI_CONFIG="$(
    find "${DEPTHAI_INSTALL}" \
        -name "depthaiConfig.cmake" \
        -print \
        -quit
)"

echo
echo "========================================"
echo " DepthAI C++ setup completed"
echo "========================================"

if [ -n "${DEPTHAI_CONFIG}" ]; then

    echo "Found:"
    echo "${DEPTHAI_CONFIG}"

else

    echo "WARNING: depthaiConfig.cmake was not found."
    exit 1
fi

echo
echo "CMAKE_PREFIX_PATH:"
echo "${DEPTHAI_INSTALL}"
echo
echo "Reconnect the OAK-D device if necessary."
