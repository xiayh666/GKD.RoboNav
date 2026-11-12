#!/bin/bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /work/catkin/devel/setup.bash
export ROS_HOME=/work/ros_home
roscore > /work/roscore-capacity.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true' EXIT
for attempt in {1..100}; do
 if rosparam list >/dev/null 2>&1; then break; fi
 sleep 0.1
done
for cap in 100000 300 301; do
 python3 /capacity/run_reference.py /input/stationary.csv /results/official_${cap}.csv --capacity "$cap"
done
