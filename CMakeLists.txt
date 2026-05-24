cmake_minimum_required(VERSION 3.10)
project(baxter_pnp_enterprise VERSION 1.0.0)

# ── C++ Standard ──────────────────────────────────────────────────────────
# C++17 required by BehaviorTree.CPP v4 (std::any, std::variant, etc.)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler Warnings (Enterprise grade: warnings = errors) ──────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wno-unused-parameter)  # Allow unused params in interfaces
endif()

# ── Catkin ────────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  tf
  tf2
  tf2_ros
  diagnostic_updater
  dynamic_reconfigure
  nodelet
  baxter_core_msgs
  baxter_interface
  baxter_maintenance_msgs
)

# ── BehaviorTree.CPP v4 ──────────────────────────────────────────────────
# If installed as a system package or catkin workspace package
find_package(behaviortree_cpp_v4 QUIET)
if(NOT behaviortree_cpp_v4_FOUND)
  # Fallback: find the standalone CMake package
  find_package(behaviortree_cpp REQUIRED)
  set(behaviortree_cpp_v4_LIBRARIES ${behaviortree_cpp_LIBRARIES})
  set(behaviortree_cpp_v4_INCLUDE_DIRS ${behaviortree_cpp_INCLUDE_DIRS})
endif()

# ── Dynamic Reconfigure ──────────────────────────────────────────────────
# Generate dynamic_reconfigure cfg header
generate_dynamic_reconfigure_options(
  cfg/BaxterPnP.cfg
)

# ── Messages / Services / Actions (if needed) ─────────────────────────────
# add_message_files(...)
# add_service_files(...)
# add_action_files(...)
# generate_messages(...)

# ── Catkin Package Setup ─────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES baxter_pnp_hardware baxter_pnp_bt_nodes baxter_pnp_main
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs sensor_msgs trajectory_msgs
    control_msgs actionlib actionlib_msgs tf tf2 tf2_ros
    diagnostic_updater dynamic_reconfigure nodelet
    baxter_core_msgs baxter_interface baxter_maintenance_msgs
  DEPENDS behaviortree_cpp_v4
)

# ── Include directories ──────────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${behaviortree_cpp_v4_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # For generated cfg headers
)

# ═══════════════════════════════════════════════════════════════════════════
#  Library: baxter_pnp_hardware
#  Everything that talks to Baxter SDK: lifecycle, IK, gripper, diagnostics
# ═══════════════════════════════════════════════════════════════════════════
add_library(baxter_pnp_hardware
  src/robot_lifecycle.cpp
  src/ik_solver.cpp
  src/gripper_controller.cpp
  src/diagnostic_updater.cpp
)

target_link_libraries(baxter_pnp_hardware
  ${catkin_LIBRARIES}
  ${behaviortree_cpp_v4_LIBRARIES}
)

# ═══════════════════════════════════════════════════════════════════════════
#  Library: baxter_pnp_bt_nodes
#  All BehaviorTree.CPP node registrations
# ═══════════════════════════════════════════════════════════════════════════
add_library(baxter_pnp_bt_nodes
  src/bt_nodes.cpp
)

target_link_libraries(baxter_pnp_bt_nodes
  baxter_pnp_hardware
  ${catkin_LIBRARIES}
  ${behaviortree_cpp_v4_LIBRARIES}
)

# ═══════════════════════════════════════════════════════════════════════════
#  Executable: pnp_main
#  Entry point with AsyncSpinner + BT execution loop
# ═══════════════════════════════════════════════════════════════════════════
add_executable(pnp_main
  src/pnp_main.cpp
)

target_link_libraries(pnp_main
  baxter_pnp_hardware
  baxter_pnp_bt_nodes
  ${catkin_LIBRARIES}
  ${behaviortree_cpp_v4_LIBRARIES}
)

# Make executable runnable with rosrun
add_dependencies(pnp_main ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ═══════════════════════════════════════════════════════════════════════════
#  Nodelet plugin (zero-copy camera transport for process isolation)
# ═══════════════════════════════════════════════════════════════════════════
add_library(baxter_pnp_nodelet
  src/camera_nodelet.cpp
)
target_link_libraries(baxter_pnp_nodelet
  ${catkin_LIBRARIES}
)
# Register nodelet plugin (generates nodelet_plugins.xml)
add_dependencies(baxter_pnp_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ── Install ──────────────────────────────────────────────────────────────
install(
  TARGETS
    baxter_pnp_hardware
    baxter_pnp_bt_nodes
    baxter_pnp_nodelet
    pnp_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
    bt_xml
    docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ── Tests ─────────────────────────────────────────────────────────────────
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_ik_solver.test)
endif()
