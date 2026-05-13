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

add_executable(cartesian_to_haptic_abb
  src/cartesianToHaptic/cartesianToHaptic_abb.cpp
  src/cartesianToHaptic/baseNode/cartesianToHaptic_baseNode.cpp
)

add_executable(cartesian_to_haptic_teo
  src/cartesianToHaptic/cartesianToHaptic_teo.cpp
  src/cartesianToHaptic/baseNode/cartesianToHaptic_baseNode.cpp
)

add_executable(haptic_to_cartesian_abb
  src/hapticToCartesian/hapticToCartesian_abb.cpp
  src/hapticToCartesian/baseNode/hapticToCartesian_baseNode.cpp
)

add_executable(haptic_to_cartesian_teo
  src/hapticToCartesian/hapticToCartesian_teo.cpp
  src/hapticToCartesian/baseNode/hapticToCartesian_baseNode.cpp
)

target_include_directories(cartesian_to_haptic_teo PUBLIC include)
target_include_directories(cartesian_to_haptic_teo PRIVATE include_priv)
ament_target_dependencies(cartesian_to_haptic_teo
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  orocos_kdl
  sensor_msgs
)

target_include_directories(cartesian_to_haptic_abb PUBLIC include)
target_include_directories(cartesian_to_haptic_abb PRIVATE include_priv)
ament_target_dependencies(cartesian_to_haptic_abb
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  orocos_kdl
  sensor_msgs
)

target_include_directories(haptic_to_cartesian_teo PUBLIC include)
target_include_directories(haptic_to_cartesian_teo PRIVATE include_priv)
ament_target_dependencies(haptic_to_cartesian_teo
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  orocos_kdl
  sensor_msgs
)

target_include_directories(haptic_to_cartesian_abb PUBLIC include)
target_include_directories(haptic_to_cartesian_abb PRIVATE include_priv)
ament_target_dependencies(haptic_to_cartesian_abb
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  orocos_kdl
  sensor_msgs
)

install(TARGETS cartesian_to_haptic_abb
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS cartesian_to_haptic_teo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS haptic_to_cartesian_abb
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS haptic_to_cartesian_teo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()