#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
WORKSPACE="${SCRIPT_DIR}/car_ws0917/car_ws0917"

source /opt/ros/noetic/setup.bash

if [[ ! -f "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "ROS workspace has not been built. Run: cd '${WORKSPACE}' && catkin_make" >&2
  exit 1
fi

source "${WORKSPACE}/devel/setup.bash"
exec roslaunch agv_bridge core.launch "$@"
