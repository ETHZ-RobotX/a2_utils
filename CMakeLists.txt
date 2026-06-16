cmake_minimum_required(VERSION 3.16)
project(a2_utils)

set(DEPENDENCY_LIST
  rclcpp
  unitree_hg
  sensor_msgs
  tf2_ros
  tf2
  tf2_sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep ${DEPENDENCY_LIST})
  find_package(${dep} REQUIRED)
endforeach()
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(a2_interfaces REQUIRED)

# ── ModeFsm library (pure C++, no ROS dep) ────────────────────────────────────
add_library(a2_mode_fsm src/mode_fsm.cpp)
target_include_directories(a2_mode_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── SafeVelocityRosInterface library ─────────────────────────────────────────
add_library(a2_safe_velocity_ros_interface src/safe_velocity_ros_interface.cpp)
target_link_libraries(a2_safe_velocity_ros_interface a2_mode_fsm)
ament_target_dependencies(a2_safe_velocity_ros_interface rclcpp geometry_msgs a2_interfaces)
target_include_directories(a2_safe_velocity_ros_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS a2_mode_fsm a2_safe_velocity_ros_interface EXPORT a2_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(a2_utilsTargets HAS_LIBRARY_TARGET)
ament_export_libraries(a2_safe_velocity_ros_interface a2_mode_fsm)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs a2_interfaces)

add_executable(registered_scan_pub src/registered_scan_pub.cpp)
ament_target_dependencies(registered_scan_pub ${DEPENDENCY_LIST})

add_executable(joint_states_pub src/joint_states_pub.cpp)
ament_target_dependencies(joint_states_pub rclcpp unitree_hg sensor_msgs)

add_executable(imu_pub src/imu_pub.cpp)
ament_target_dependencies(imu_pub rclcpp unitree_hg sensor_msgs)

add_executable(cmd_vel_sport_relay src/cmd_vel_sport_relay.cpp)
ament_target_dependencies(cmd_vel_sport_relay rclcpp geometry_msgs unitree_api)

install(TARGETS
  registered_scan_pub
  joint_states_pub
  imu_pub
  cmd_vel_sport_relay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
