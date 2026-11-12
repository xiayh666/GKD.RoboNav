cmake_minimum_required(VERSION 3.18)
project(terrain_map LANGUAGES NONE)

# Plain CMake entry point for colcon; the library remains a Cargo crate.
find_program(CARGO_EXECUTABLE cargo
    HINTS "$ENV{CARGO_HOME}/bin" "$ENV{HOME}/.cargo/bin"
    REQUIRED
)
set(TERRAIN_MAP_CARGO_PROFILE release CACHE STRING "Cargo build profile")
set(TERRAIN_MAP_CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo" CACHE PATH
    "Cargo artifact directory")

# Invoke Cargo on every build so that it tracks Rust sources and dependencies.
# Cargo itself skips compilation when its inputs have not changed.
add_custom_target(terrain_map_cargo ALL
    # CMake does not forward make jobserver descriptors to this command.
    COMMAND "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS
        "${CARGO_EXECUTABLE}" build
        --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
        --package terrain-map --lib --locked
        --profile "${TERRAIN_MAP_CARGO_PROFILE}"
        --target-dir "${TERRAIN_MAP_CARGO_TARGET_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    USES_TERMINAL
    VERBATIM
)

# Rust consumers use Cargo path dependencies, not installed standalone rlibs.
install(FILES README.md DESTINATION share/${PROJECT_NAME})
