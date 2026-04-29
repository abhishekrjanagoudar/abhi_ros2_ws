cmake_minimum_required(VERSION 3.8)
project(mobile_description)

# =============================================================================
# Build Configuration — mobile_description
# =============================================================================
# Package layout:
#   launch/   — Launch files (rsp, sim, gazebo, rviz)
#   src/      — Top-level orchestrator launchers (robot.launch.py)
#   config/   — Controllers YAML, ros_gz_bridge YAML, robot URDF, env worlds/models
#   rviz/     — Default RViz configurations
# =============================================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# ── Install Resources ────────────────────────────────────────────────────────
# Each directory ships to: install/mobile_description/share/mobile_description/<dir>
install(DIRECTORY
  launch
  src
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
