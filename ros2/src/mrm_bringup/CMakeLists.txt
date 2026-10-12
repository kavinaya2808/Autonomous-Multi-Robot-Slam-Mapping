cmake_minimum_required(VERSION 3.8)
project(mrm_bringup)

find_package(ament_cmake REQUIRED)

# Nothing to compile: just copy these folders to share/mrm_bringup/ so that
# $(find-pkg-share mrm_bringup) can locate them at runtime.
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
