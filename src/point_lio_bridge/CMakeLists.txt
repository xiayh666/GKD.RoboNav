cmake_minimum_required(VERSION 3.18)
project(point_lio_bridge LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Threads REQUIRED)
find_program(CARGO_EXECUTABLE cargo HINTS "$ENV{CARGO_HOME}/bin" "$ENV{HOME}/.cargo/bin" REQUIRED)
set(FFI_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/cargo/release/libpoint_lio_ffi.a")
add_custom_target(point_lio_ffi_build ALL
  COMMAND "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS
    "${CARGO_EXECUTABLE}" build --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
    --package point-lio-ffi --lib --release --locked
    --target-dir "${CMAKE_CURRENT_BINARY_DIR}/cargo"
  BYPRODUCTS "${FFI_LIBRARY}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  USES_TERMINAL VERBATIM)
add_executable(point_lio_node src/node.cpp)
add_dependencies(point_lio_node point_lio_ffi_build)
target_compile_features(point_lio_node PRIVATE cxx_std_17)
target_compile_options(point_lio_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(point_lio_node PRIVATE include)
ament_target_dependencies(point_lio_node rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2_ros livox_ros_driver2)
target_link_libraries(point_lio_node "${FFI_LIBRARY}" Threads::Threads ${CMAKE_DL_LIBS} m)
install(TARGETS point_lio_node DESTINATION lib/${PROJECT_NAME})
ament_package()
