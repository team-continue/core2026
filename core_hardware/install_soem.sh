#!/bin/bash

# Move to the directory where this script exists.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

SOEM_DIR="vendor/soem"

if [ -d "$SOEM_DIR" ]; then
    echo "SOEM is already installed."
    exit 0
fi

mkdir -p "$SOEM_DIR"
curl -L -o "$SOEM_DIR/soem-2.0.0-Linux-x86_64.zip" \
    "https://github.com/OpenEtherCATsociety/SOEM/releases/download/v2.0.0/soem-2.0.0-Linux-x86_64.zip"
unzip "$SOEM_DIR/soem-2.0.0-Linux-x86_64.zip" -d "$SOEM_DIR"
mv "$SOEM_DIR/soem-2.0.0-Linux-x86_64/"* "$SOEM_DIR/"
rm -f "$SOEM_DIR/soem-2.0.0-Linux-x86_64.zip"
rm -rf "$SOEM_DIR/soem-2.0.0-Linux-x86_64"
