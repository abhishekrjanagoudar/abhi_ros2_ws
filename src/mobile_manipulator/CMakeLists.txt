cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts as executable ROS 2 nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/fk.py
  scripts/ik.py
  scripts/arm_planner.py
  scripts/arm_teleop.py
  scripts/arm_joints.py
  scripts/pick_place_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all resource directories into the share location
install(DIRECTORY
  launch
  urdf
  config
  moveit_config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Joint-name parity test (URDF <-> controllers <-> SRDF <-> arm teleop)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_joint_name_parity
    test/test_joint_name_parity.py
    TIMEOUT 120
  )

  # Property-based test for arm-teleop joint clamping (Req 8.3); uses Hypothesis
  ament_add_pytest_test(test_arm_teleop_clamp
    test/test_arm_teleop_clamp.py
    TIMEOUT 120
  )
endif()

ament_package()
