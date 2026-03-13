#!/usr/bin/env bash

set -euo pipefail

SERVICE_NAME="core_hardware_daemon.service"
DAEMON_NAME="core_hardware_daemon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$(pwd)"
BUILD_DAEMON="$WORKSPACE_DIR/build/core_hardware/$DAEMON_NAME"
INSTALL_DAEMON="/opt/core_hardware/bin/$DAEMON_NAME"
IF_NAME="enp2s0"
SOCKET_PATH="/tmp/core_hardware.sock"
SERVICE_TEMPLATE="$PACKAGE_DIR/systemd/$SERVICE_NAME"
SERVICE_DEST="/etc/systemd/system/$SERVICE_NAME"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root." >&2
  exit 1
fi

if [[ ! -d "$WORKSPACE_DIR/src" || ! -d "$WORKSPACE_DIR/build" ]]; then
  echo "Run this script from the workspace root." >&2
  echo "Current directory: $WORKSPACE_DIR" >&2
  exit 1
fi

if [[ ! -x "$BUILD_DAEMON" ]]; then
  echo "Built daemon not found: $BUILD_DAEMON" >&2
  echo "Run: colcon build --packages-select core_hardware" >&2
  exit 1
fi

install -D -m 0755 "$BUILD_DAEMON" "$INSTALL_DAEMON"

if [[ -f "$SERVICE_DEST" ]]; then
  systemctl restart core_hardware_daemon
  echo "Updated $INSTALL_DAEMON and restarted core_hardware_daemon"
  exit 0
fi

if [[ ! -f "$SERVICE_TEMPLATE" ]]; then
  echo "Service template not found: $SERVICE_TEMPLATE" >&2
  exit 1
fi

sed \
  -e "s|@CORE_HARDWARE_DAEMON_PATH@|$INSTALL_DAEMON|g" \
  -e "s|@CORE_HARDWARE_IF_NAME@|$IF_NAME|g" \
  -e "s|@CORE_HARDWARE_SOCKET_PATH@|$SOCKET_PATH|g" \
  "$SERVICE_TEMPLATE" > "$SERVICE_DEST"
chmod 0644 "$SERVICE_DEST"

systemctl daemon-reload

echo "Installed $SERVICE_DEST"
echo "Installed $INSTALL_DAEMON"
echo "  sudo systemctl enable --now core_hardware_daemon"
