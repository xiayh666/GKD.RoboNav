#!/bin/bash
# Run inside the built reference container with /work and /comparison mounts.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /work/catkin/devel/setup.bash
roscore > /work/roscore.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true' EXIT
for attempt in {1..100}; do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.1
done
for name in rotation_control rotation_clipped stationary floor recovery timing_1000 timing_10000 timing_20000; do
    extra=()
    if [[ "$name" == rotation_clipped ]]; then extra+=(--gyro-limit 0.3); fi
    if [[ "$name" == timing_* ]]; then extra+=(--timing); fi
    python3 /scripts/run_reference.py "/comparison/$name.csv" "/comparison/official_$name.csv" "${extra[@]}"
done
