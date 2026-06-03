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

add_executable(registered_scan_pub src/registered_scan_pub.cpp)
ament_target_dependencies(registered_scan_pub ${DEPENDENCY_LIST})

add_executable(joint_states_pub src/joint_states_pub.cpp)
ament_target_dependencies(joint_states_pub rclcpp unitree_hg sensor_msgs)

add_executable(imu_pub src/imu_pub.cpp)
ament_target_dependencies(imu_pub rclcpp unitree_hg sensor_msgs)

install(TARGETS
  registered_scan_pub
  joint_states_pub
  imu_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
