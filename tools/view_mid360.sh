#!/usr/bin/env bash
# Open the official Livox RViz display through the project driver launch.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
container=${LIVOX_CONTAINER:-ros2}
if [[ -z ${XAUTHORITY:-} || ! -r "$XAUTHORITY" ]]; then
  echo 'Run from the graphical desktop terminal with readable XAUTHORITY.' >&2
  exit 1
fi
if docker exec "$container" pgrep -x livox_ros_drive >/dev/null; then
  echo 'A Livox driver is already running; stop that launch before starting another.' >&2
  exit 1
fi
docker cp "$XAUTHORITY" "$container:/home/ubuntu/.Xauthority"
docker exec -e "DISPLAY=${DISPLAY:-:0}" -e XAUTHORITY=/home/ubuntu/.Xauthority \
  -w /workspace/GKD.RoboNav "$container" bash -lc '
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    exec ros2 launch gkd_nav_bringup mid360_rviz.launch.py
  '
