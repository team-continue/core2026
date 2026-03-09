#!/usr/bin/env bash
set -e

SOEM_VERSION="v2.0.0"
REPO_USER="OpenEtherCATsociety"
REPO_NAME="SOEM"
DEST_DIR="soem"

# Detect OS and ARCH
OS="$(uname -s)"
ARCH="$(uname -m)"

echo "=== Download SOEM Release ${SOEM_VERSION} ==="
echo "Detected OS: ${OS}"
echo "Detected ARCH: ${ARCH}"

# Normalize
case "${OS}" in
    Linux) OS_ID="Linux";;
    Darwin) OS_ID="Darwin";;
    *) echo "Unsupported OS: ${OS}"; exit 1;;
esac

case "${ARCH}" in
    x86_64) ARCH_ID="x86_64";;
    aarch64) ARCH_ID="arm64";;
    armv7l | armhf) ARCH_ID="armhf";;
    *) echo "Unsupported ARCH: ${ARCH}"; exit 1;;
esac

ZIP_NAME="soem-${SOEM_VERSION#v}-${OS_ID}-${ARCH_ID}.zip"
DOWNLOAD_URL="https://github.com/${REPO_USER}/${REPO_NAME}/releases/download/${SOEM_VERSION}/${ZIP_NAME}"

echo "Download URL: ${DOWNLOAD_URL}"

# Temporary file
TMP_ZIP="/tmp/${ZIP_NAME}"

echo "[INFO] Downloading..."
wget -q --show-progress -O "${TMP_ZIP}" "${DOWNLOAD_URL}"

echo "[INFO] Extracting..."
unzip -q "${TMP_ZIP}" -d /tmp

# The extracted directory name is usually like "soem-${version}"
EXTRACT_DIR="/tmp/soem-${SOEM_VERSION#v}-${OS_ID}-${ARCH_ID}"

if [ ! -d "${EXTRACT_DIR}" ]; then
    # Fallback: sometimes it extracts into different folder
    EXTRACT_DIR="$(dirname "${TMP_ZIP}")/soem-${SOEM_VERSION#v}"
fi

if [ ! -d "${EXTRACT_DIR}" ]; then
    echo "[ERROR] Extracted directory not found!"
    exit 1
fi

echo "[INFO] Moving to ./${DEST_DIR} ..."
rm -rf "${DEST_DIR}"
mv "${EXTRACT_DIR}" "${DEST_DIR}"

echo "[INFO] Cleaning up..."
rm -f "${TMP_ZIP}"

echo "=== SOEM ${SOEM_VERSION} installed into ./${DEST_DIR} ==="
