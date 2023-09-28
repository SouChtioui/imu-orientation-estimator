cmake_minimum_required(VERSION 3.8)
project(tilt_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tilt_detection_msgs REQUIRED)

add_library(rotation_estimation src/tools/IMUComplementaryFilter.cpp src/tools/OrientationEstimator.cpp)
target_include_directories(rotation_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  rotation_estimation
  "rclcpp"
  "sensor_msgs"
)

add_executable(tilt_detection_node src/tilt_detection_node.cpp)
ament_target_dependencies(
  rotation_estimation
  "tf2"
  "tilt_detection_msgs"
)
target_link_libraries(
  tilt_detection_node
  "rotation_estimation"
)

install(TARGETS rotation_estimation
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS tilt_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
