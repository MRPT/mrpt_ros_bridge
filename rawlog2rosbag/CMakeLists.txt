cmake_minimum_required(VERSION 3.10)
project(rawlog2rosbag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CLI11 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_maps 3.0 REQUIRED)
find_package(mrpt_obs 3.0 REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall)
endif()

add_executable(${PROJECT_NAME} src/rawlog2rosbag.cpp)

target_link_libraries(${PROJECT_NAME}
  mrpt_libros_bridge::mrpt_libros_bridge
  CLI11::CLI11
  mrpt::mrpt_maps
  mrpt::mrpt_obs
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs_library
  std_msgs::std_msgs__rosidl_typesupport_cpp
  tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
)

# rosbag2 before Iron (Humble) stores the topic QoS profiles as a YAML string,
# and names the message stamp differently:
if(rosbag2_storage_VERSION VERSION_LESS "0.17.0")
  set(ROSBAG2_PRE_IRON 1)
else()
  set(ROSBAG2_PRE_IRON 0)
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSBAG2_PRE_IRON=${ROSBAG2_PRE_IRON})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Round trip: synthetic rawlog -> rawlog2rosbag -> read the bag back -> compare.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/round_trip_unittest.cpp
    ENV RAWLOG2ROSBAG_EXE=$<TARGET_FILE:${PROJECT_NAME}>
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      mrpt_libros_bridge::mrpt_libros_bridge
      mrpt::mrpt_obs
      mrpt::mrpt_maps
      rosbag2_cpp::rosbag2_cpp
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      std_msgs::std_msgs__rosidl_typesupport_cpp
      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    )
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE ROSBAG2_PRE_IRON=${ROSBAG2_PRE_IRON})
  endif()
endif()

ament_package()
