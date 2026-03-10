#!/usr/bin/env bash

set -euo pipefail

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
CORE_WS=${CORE_WS:-$HOME/core_ws}
LIVOX_WS=${LIVOX_WS:-$HOME/Livox_ws}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$CORE_WS/install/setup.bash" ]]; then
  echo "core_ws setup not found: $CORE_WS/install/setup.bash" >&2
  exit 1
fi

if [[ ! -f "$LIVOX_WS/install/setup.bash" ]]; then
  echo "Livox_ws setup not found: $LIVOX_WS/install/setup.bash" >&2
  exit 1
fi

# ROS 2 setup scripts are not always nounset-safe.
set +u
source "$ROS_SETUP"
source "$CORE_WS/install/setup.bash"
source "$LIVOX_WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

exec ros2 launch core_launch navigation.launch.py odom_source:=fastlio "$@"