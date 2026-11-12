#!/bin/bash
# Run inside the Noetic image; official sources are mounted read-only at /official.
set -euo pipefail
source /opt/ros/noetic/setup.bash
mkdir -p /work/catkin/src/livox_ros_driver/msg
mkdir -p /work/catkin/src/point_lio
cp -a /official/. /work/catkin/src/point_lio/
cp /livox_messages/*.msg /work/catkin/src/livox_ros_driver/msg/
cat > /work/catkin/src/livox_ros_driver/package.xml <<'XML'
<package format="2"><name>livox_ros_driver</name><version>0.0.0</version>
<description>Official Livox messages only, for offline reference replay</description>
<maintainer email="offline@example.invalid">offline</maintainer><license>MIT</license>
<buildtool_depend>catkin</buildtool_depend><build_depend>message_generation</build_depend>
<depend>std_msgs</depend><exec_depend>message_runtime</exec_depend></package>
XML
cat > /work/catkin/src/livox_ros_driver/CMakeLists.txt <<'CMAKE'
cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES CustomMsg.msg CustomPoint.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
CMAKE
cd /work/catkin
catkin_make --force-cmake -j1 -l1 -DPYTHON_EXECUTABLE=/usr/bin/python3
test -x /work/catkin/devel/lib/point_lio/pointlio_mapping
