cmake_minimum_required(VERSION 3.8)
project(mrm_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 1) The algorithms (frontier detection, map saving): plain C++, no ROS.
#    STATIC = compiled straight into the programs that use it.
add_library(frontier_detector STATIC src/frontier_detector.cpp src/map_io.cpp)
target_include_directories(frontier_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(frontier_detector PUBLIC cxx_std_17)

# 2) The ROS node that uses it.
add_executable(frontier_explorer src/frontier_explorer_node.cpp)
target_link_libraries(frontier_explorer frontier_detector)
ament_target_dependencies(frontier_explorer
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs std_msgs visualization_msgs tf2 tf2_ros)

install(TARGETS frontier_explorer
  DESTINATION lib/${PROJECT_NAME})

# 3) Unit tests (colcon test). They only need the library, not ROS.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frontier_detector test/test_frontier_detector.cpp)
  target_link_libraries(test_frontier_detector frontier_detector)
  ament_add_gtest(test_map_io test/test_map_io.cpp)
  target_link_libraries(test_map_io frontier_detector)
endif()

ament_package()
