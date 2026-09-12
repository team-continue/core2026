#!/usr/bin/env bash
set -euo pipefail

# Pin the same DepthAI 3.9.0 revision for local builds and CI.
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="${PACKAGE_DIR}/third_party"
DEPTHAI_SRC="${THIRD_PARTY}/depthai-core"
DEPTHAI_BUILD="${THIRD_PARTY}/depthai-build"
DEPTHAI_INSTALL="${THIRD_PARTY}/depthai_install"
DEPTHAI_REF="ac928141ef7750bf3082b5e1c319a115618660e7"

# CI only compiles this package; no OAK hardware is started there.
# Set DEPTHAI_CI_BUILD=0 to build a hardware-capable SDK even on CI.
DEPTHAI_CI_BUILD="${DEPTHAI_CI_BUILD:-${CI:-false}}"
RESOURCE_OPTIONS=()
if [[ "${DEPTHAI_CI_BUILD}" == true || "${DEPTHAI_CI_BUILD}" == 1 ]]; then
    RESOURCE_OPTIONS=(
        -DDEPTHAI_ENABLE_DEVICE_FW=OFF
        -DDEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW=OFF
        -DDEPTHAI_ENABLE_DEVICE_RVC3_FW=OFF
        -DDEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF
        -DDEPTHAI_EMBED_FRONTEND=OFF
    )
else
    # Restore defaults when reusing a build directory previously used for CI.
    RESOURCE_OPTIONS=(
        -DDEPTHAI_ENABLE_DEVICE_FW=ON
        -DDEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW=ON
        -DDEPTHAI_ENABLE_DEVICE_RVC3_FW=OFF
        -DDEPTHAI_ENABLE_DEVICE_RVC4_FW=ON
        -DDEPTHAI_EMBED_FRONTEND=ON
    )
fi
INSTALL_KEY="$(sha256sum "${BASH_SOURCE[0]}" | cut -d ' ' -f 1)-${DEPTHAI_CI_BUILD}"
STAMP="${DEPTHAI_INSTALL}/.installer-key"
if [[ -f "${DEPTHAI_INSTALL}/lib/cmake/depthai/depthaiConfig.cmake" &&
      -f "${DEPTHAI_INSTALL}/lib/libdepthai-core.so" && -f "${STAMP}" ]] &&
   [[ "$(cat "${STAMP}")" == "${INSTALL_KEY}" ]]; then
    echo "DepthAI SDK already installed; skipping setup and build."
    exit 0
fi

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
    -DDEPTHAI_BUILD_PYTHON=OFF \
    "${RESOURCE_OPTIONS[@]}"
cmake --build "${DEPTHAI_BUILD}" --parallel "${DEPTHAI_BUILD_JOBS:-2}"
cmake --install "${DEPTHAI_BUILD}"

test -f "${DEPTHAI_INSTALL}/lib/cmake/depthai/depthaiConfig.cmake"
printf '%s\n' "${INSTALL_KEY}" > "${STAMP}"
echo "DepthAI installed: ${DEPTHAI_INSTALL}"
# USB/udev configuration is a separate hardware setup step, unnecessary in CI.
