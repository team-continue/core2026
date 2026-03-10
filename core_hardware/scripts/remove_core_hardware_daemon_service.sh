#!/usr/bin/env bash

set -euo pipefail

SERVICE_NAME="core_hardware_daemon.service"
SERVICE_UNIT="core_hardware_daemon"
SERVICE_DEST="/etc/systemd/system/$SERVICE_NAME"
INSTALL_DAEMON="/opt/core_hardware/bin/core_hardware_daemon"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root." >&2
  exit 1
fi

if systemctl list-unit-files "$SERVICE_NAME" >/dev/null 2>&1; then
  systemctl disable --now "$SERVICE_UNIT" >/dev/null 2>&1 || true
fi

rm -f "$SERVICE_DEST"
rm -f "$INSTALL_DAEMON"

systemctl daemon-reload

echo "Removed $SERVICE_DEST"
echo "Removed $INSTALL_DAEMON"
