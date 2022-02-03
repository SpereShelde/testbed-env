cmake_minimum_required(VERSION 3.5)
project(robot_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roslib REQUIRED)
# find_package(tf2 REQUIRED)                       
find_package(robot_msg REQUIRED)                         
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

set(robot_core_SRCS
  src/robot_core.cpp
  # src/Quaternion_Solution.cpp
)

add_executable(core_node src/robot_core.cpp)
ament_target_dependencies(core_node rclcpp serial robot_msg)

install(TARGETS
core_node
#parameter_node

DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
