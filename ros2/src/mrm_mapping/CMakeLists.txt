cmake_minimum_required(VERSION 3.8)
project(mrm_mapping)

# Turn on warnings: in C++ they catch real bugs (unused values, sign mix-ups...).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Each find_package() loads the CMake config that the dependency installed.
# They must match the <depend> lines in package.xml.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# One executable = one node program.
add_executable(map_merger src/map_merger.cpp)
target_compile_features(map_merger PUBLIC cxx_std_17)
ament_target_dependencies(map_merger
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install to lib/<package> so that `ros2 run mrm_mapping map_merger` finds it.
install(TARGETS map_merger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
