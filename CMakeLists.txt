cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2_intel_d435)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Find ROS 2 packages
find_package(apriltag_detector QUIET)

# Uncomment if you have other ament_cmake packages
# find_package(apriltag_msgs QUIET)
# find_package(apriltag_ros QUIET)

# This is a metapackage
ament_cmake_core_register_metapackage()

# Find and register all packages
if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)

  ament_copyright()
  ament_flake8(EXCLUDE build install)
  ament_pep257(EXCLUDE build install)
endif()

ament_package()

