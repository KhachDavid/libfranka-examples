cmake_minimum_required(VERSION 3.20)
project(move_joint7 LANGUAGES CXX)

# ─── Build settings ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── User-selectable prefix for libfranka ──────────────────────────────────────
#   Pass -DFRANKA_INSTALL=<prefix> (or set ENV{FRANKA_INSTALL}) to point CMake
#   at your libfranka installation.  Falls back to the build-in path so the file
#   still works out-of-the-box on your machine.
set(FRANKA_INSTALL
    "$ENV{FRANKA_INSTALL}"                # 1) environment variable
    CACHE PATH "Path to libfranka installation prefix")

if(FRANKA_INSTALL)
  list(APPEND CMAKE_PREFIX_PATH "${FRANKA_INSTALL}")
endif()

# ─── Dependencies ─────────────────────────────────────────────────────────────
find_package(Franka CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# ─── Helper library (shared by all demos) ─────────────────────────────────────
add_library(common STATIC src/examples_common.cpp)
target_include_directories(common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(common Franka::Franka Eigen3::Eigen)

# ─── Executables ──────────────────────────────────────────────────────────────
set(EXAMPLES
  move_joint7_position
  move_joint7_velocity
  move_joint7_pose
  move_joint7_twist
  move_joint7_torque
  sudden_torque_drop)

foreach(ex ${EXAMPLES})
  add_executable(${ex} src/${ex}.cpp)
  target_link_libraries(${ex} common)
endforeach()

# ─── Install targets (optional) ───────────────────────────────────────────────
install(TARGETS ${EXAMPLES} RUNTIME DESTINATION bin)
