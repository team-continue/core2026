#!/usr/bin/env bash
set -euo pipefail

# Pin the same DepthAI 3.9.0 revision for local builds and CI.
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="${PACKAGE_DIR}/third_party"
DEPTHAI_SRC="${THIRD_PARTY}/depthai-core"
DEPTHAI_BUILD="${THIRD_PARTY}/depthai-build"
DEPTHAI_INSTALL="${THIRD_PARTY}/depthai_install"
DEPTHAI_REF="ac928141ef7750bf3082b5e1c319a115618660e7"

# CI runs as root; local machines generally need sudo.
if [[ "${DEPTHAI_SKIP_APT:-0}" != 1 ]]; then
    SUDO=()
    if (( EUID != 0 )); then
        SUDO=(sudo)
    fi
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y \
        build-essential cmake git pkg-config libudev-dev libusb-1.0-0-dev \
        libopencv-dev curl zip unzip tar ninja-build ca-certificates
fi

mkdir -p "${THIRD_PARTY}"
touch "${THIRD_PARTY}/COLCON_IGNORE"
if [[ ! -d "${DEPTHAI_SRC}/.git" ]]; then
    git clone https://github.com/luxonis/depthai-core.git "${DEPTHAI_SRC}"
fi
if [[ -n "$(git -C "${DEPTHAI_SRC}" status --porcelain)" ]]; then
    echo "ERROR: ${DEPTHAI_SRC} has local changes; leaving them untouched." >&2
    exit 1
fi
if ! git -C "${DEPTHAI_SRC}" cat-file -e "${DEPTHAI_REF}^{commit}"; then
    git -C "${DEPTHAI_SRC}" fetch origin "${DEPTHAI_REF}"
fi
git -C "${DEPTHAI_SRC}" checkout --detach "${DEPTHAI_REF}"
git -C "${DEPTHAI_SRC}" submodule update --init --recursive

# Never delete an existing build directory; CMake supports incremental builds.
cmake -S "${DEPTHAI_SRC}" -B "${DEPTHAI_BUILD}" \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${DEPTHAI_INSTALL}" \
    -DDEPTHAI_BUILD_EXAMPLES=OFF \
    -DDEPTHAI_BUILD_TESTS=OFF \
    -DDEPTHAI_BUILD_PYTHON=OFF
cmake --build "${DEPTHAI_BUILD}" --parallel "${DEPTHAI_BUILD_JOBS:-2}"
cmake --install "${DEPTHAI_BUILD}"

test -f "${DEPTHAI_INSTALL}/lib/cmake/depthai/depthaiConfig.cmake"
echo "DepthAI installed: ${DEPTHAI_INSTALL}"
# USB/udev configuration is a separate hardware setup step, unnecessary in CI.
