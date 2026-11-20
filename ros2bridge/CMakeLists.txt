cmake_minimum_required(VERSION 3.10)
project(mrpt_libros_bridge)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required to detect ROS 2 environment
find_package(ament_cmake REQUIRED)

# Find dependencies listed in package.xml
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosbag2_cpp QUIET)  # optional
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
if(rosbag2_cpp_FOUND)
  set(rosbag2_cpp_lib       rosbag2_cpp::rosbag2_cpp)
  set(rosbag2_cpp_cmake_dep rosbag2_cpp)
endif()

# Find MRPT dependencies
find_package(mrpt_maps 3.0 REQUIRED)
find_package(mrpt_img 3.3 REQUIRED)
find_package(mrpt_obs 3.3 REQUIRED)

# Define the "lib${PROJECT_NAME}.so" shared library
add_library(${PROJECT_NAME} SHARED
  src/time.cpp
  src/pose.cpp
  src/laser_scan.cpp
  src/map.cpp
  src/gps.cpp
  src/range.cpp
  src/imu.cpp
  src/point_cloud2.cpp
  src/image.cpp
  src/stereo_image.cpp
  src/camera_info.cpp
  src/odometry.cpp
  src/ros_to_mrpt_obs.cpp
  src/rosbag2_to_mrpt.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rosbag2_cpp_lib}
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  mrpt::mrpt_maps
  mrpt::mrpt_obs
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs_library
  gps_msgs::gps_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  stereo_msgs::stereo_msgs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2::tf2
  rmw::rmw
  rmw_implementation::rmw_implementation
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)

# Installation rules
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export package information
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  sensor_msgs
  gps_msgs
  nav_msgs
  stereo_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  rmw
  rmw_implementation
  rosidl_typesupport_cpp
  ${rosbag2_cpp_cmake_dep}
  mrpt_maps
  mrpt_obs
)

# Unit tests
if(BUILD_TESTING)
  # find GTest and ament_cmake_gtest
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED NO_MODULE)

  ament_add_gtest(test_${PROJECT_NAME}
    test/camera_info_unittest.cpp
    test/gps_unittest.cpp
    test/image_unittest.cpp
    test/imu_unittest.cpp
    test/laser_scan_unittest.cpp
    test/map_unittest.cpp
    test/pointcloud2_unittest.cpp
    test/pose_unittest.cpp
    test/rosbag2_to_mrpt_unittest.cpp
    test/time_unittest.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      Eigen3::Eigen
    )
  endif()
endif()

ament_package()

# === Export a standard CMake config file for backwards compatibility from MRPT2 ===
include(CMakePackageConfigHelpers)

# Create an alias target so users can link with mrpt::ros2bridge
add_library(mrpt::ros2bridge ALIAS ${PROJECT_NAME})

# Project version: autodetect from caller package.xml
# Example line:" <version>0.3.2</version>"
file(READ ${CMAKE_SOURCE_DIR}/package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MRPT_ROS2BRIDGE_VERSION ${CMAKE_MATCH_1})
message(STATUS "MRPT_ROS2BRIDGE_VERSION version: ${MRPT_ROS2BRIDGE_VERSION} (detected in package.xml)")

# Generate a simple config file that re-exports the target
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mrpt-ros2bridge-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config.cmake"
  INSTALL_DESTINATION lib/cmake/mrpt-ros2bridge
)

# Generate an accompanying version file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config-version.cmake"
  VERSION ${MRPT_ROS2BRIDGE_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install both config and version files
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros2bridge-config-version.cmake"
  DESTINATION lib/cmake/mrpt-ros2bridge
)
