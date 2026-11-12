#!/usr/bin/env bash
# Run on host; compile/install the pinned official SDK inside the existing ros2 container.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_dir"
container=${LIVOX_CONTAINER:-ros2}
sdk_revision=c0796f04c143143899c87a773d9f6b7136453c0b
sdk_source=build/livox-vendor/Livox-SDK2
git submodule update --init -- src/livox_ros_driver2
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
mkdir -p build/livox-vendor
touch build/livox-vendor/COLCON_IGNORE
if [[ ! -d "$sdk_source/.git" ]]; then
  git clone --no-checkout https://github.com/Livox-SDK/Livox-SDK2.git "$sdk_source"
  git -C "$sdk_source" checkout --detach "$sdk_revision"
fi
if [[ $(git -C "$sdk_source" rev-parse HEAD) != "$sdk_revision" ]] ||
   [[ -n $(git -C "$sdk_source" status --porcelain) ]]; then
  echo "SDK checkout differs from pinned revision; inspect $sdk_source before continuing." >&2
  exit 1
fi
# The container must mount this project at /workspace/GKD.RoboNav and use host networking.
docker exec -w /workspace/GKD.RoboNav "$container" bash -lc '
  cmake -S build/livox-vendor/Livox-SDK2 -B build/livox-sdk2 -DCMAKE_BUILD_TYPE=Release
  cmake --build build/livox-sdk2 --parallel 2
'
docker exec -u 0 -w /workspace/GKD.RoboNav "$container" bash -lc '
  cmake --install build/livox-sdk2
  ldconfig
'
docker exec -e MAKEFLAGS=-j2 -w /workspace/GKD.RoboNav "$container" bash -lc '
  source /opt/ros/jazzy/setup.bash
  colcon build \
    --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
'
