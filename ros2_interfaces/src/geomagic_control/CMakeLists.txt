cmake_minimum_required(VERSION 3.8)
project(geomagic_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(haptic_to_cartesian
  src/hapticToCartesian/hapticToCartesianNode.cpp
)
target_include_directories(haptic_to_cartesian PUBLIC include)
ament_target_dependencies(haptic_to_cartesian
  rclcpp
  geometry_msgs
  std_msgs
  orocos_kdl
)

add_executable(cartesian_to_haptic
  src/cartesianToHaptic/cartesianToHapticFeedbackNode.cpp
)
target_include_directories(cartesian_to_haptic PUBLIC include)
ament_target_dependencies(cartesian_to_haptic
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  orocos_kdl
  sensor_msgs
)

install(TARGETS haptic_to_cartesian
  cartesian_to_haptic
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()