cmake_minimum_required(VERSION 3.8)
project(orientation_viz)

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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_include_directories(tf_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  tf_broadcaster
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
)

install(TARGETS tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
