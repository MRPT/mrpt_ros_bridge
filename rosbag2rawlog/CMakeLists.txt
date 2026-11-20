cmake_minimum_required(VERSION 3.10)
project(rosbag2rawlog)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)

find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_maps 3.0 REQUIRED)
find_package(mrpt_containers 3.0 REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
	add_compile_options(-Wall)
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

######################
## rosbag2rawlog    ##
######################
find_package(CLI11 REQUIRED)

add_executable(${PROJECT_NAME}
              src/rosbag2rawlog.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  mrpt_libros_bridge::mrpt_libros_bridge
  CLI11::CLI11
  mrpt::mrpt_maps
  mrpt::mrpt_containers
  rclcpp::rclcpp
  tf2::tf2
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs_library
  rosbag2_cpp::rosbag2_cpp
  tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  #mrpt_msgs::mrpt_msgs_library  # Change when mrpt_msgs >=0.6.0 is released
  mrpt_msgs::mrpt_msgs__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION bin
)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
