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
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Find MRPT dependency (from mrpt_libapps)
find_package(mrpt-apps REQUIRED)

# find GTest and ament_cmake_gtest
find_package(ament_cmake_gtest REQUIRED)

# Define the "lib${PROJECT_NAME}.so" shared library
add_library(${PROJECT_NAME} SHARED
  src/time.cpp
  src/pose.cpp
  src/laser_scan.cpp
  src/map.cpp
  src/gps.cpp
  src/range.cpp
  src/imu.cpp
  src/point_cloud.cpp
  src/point_cloud2.cpp
  src/image.cpp
  src/stereo_image.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  #rclcpp::rclcpp
  #rosbag2_cpp::rosbag2_cpp
  #tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
  cv_bridge::cv_bridge
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  mrpt::apps
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs_library
  std_msgs::std_msgs__rosidl_typesupport_cpp
  stereo_msgs::stereo_msgs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2::tf2
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
ament_export_dependencies(
  geometry_msgs
  sensor_msgs
  nav_msgs
  stereo_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
  mrpt-apps
)

# Unit tests
if(BUILD_TESTING)
  ament_add_gtest(test_${PROJECT_NAME}
    src/map_unittest.cpp
    src/pointcloud2_unittest.cpp
    src/pose_unittest.cpp
    src/time_unittest.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
    )
  endif()
endif()

ament_package()


