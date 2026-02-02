cmake_minimum_required(VERSION 3.8)
project(geomagic_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yarp_control_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(geomagic_control_node
  src/hapticToCartesianNode.cpp
  src/cartesianToHapticFeedbackNode.cpp
  src/main.cpp
)

target_include_directories(geomagic_control_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${orocos_kdl_INCLUDE_DIRS}
)

# Link con dependencias de ROS2
ament_target_dependencies(geomagic_control_node
  rclcpp
  geometry_msgs
  std_msgs
  orocos_kdl
  sensor_msgs
  yarp_control_msgs
)

target_link_libraries(geomagic_control_node
  yarp_control_msgs::yarp_control_msgs__rosidl_typesupport_cpp
)

install(TARGETS
  geomagic_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
