
project(mrpt_ros1bridge LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)


# Additional dependencies for tests only:
if (pcl_conversions_FOUND AND PCL_FOUND)
  set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEPS ${pcl_conversions_LIBRARIES} ${PCL_COMMON_LIBRARIES})
  set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_INCLUDE_DIRS ${pcl_conversions_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS})
  set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEFINITIONS "HAVE_PCL_CONVERSIONS")
endif()

# define lib:
set(LIB_SRCS
  src/pose_unittest.cpp
  src/stereo_image.cpp
  src/laser_scan.cpp
  src/map.cpp
  src/time.cpp
  src/time_unittest.cpp
  src/gps.cpp
  src/imu.cpp
  src/pose.cpp
  src/map_unittest.cpp
  src/image.cpp
  src/point_cloud2.cpp
  src/range.cpp
  src/pointcloud2_unittest.cpp
  src/point_cloud.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/ros1bridge
  include/mrpt/ros1bridge/pose.h
  include/mrpt/ros1bridge/time.h
  include/mrpt/ros1bridge/range.h
  include/mrpt/ros1bridge/stereo_image.h
  include/mrpt/ros1bridge/map.h
  include/mrpt/ros1bridge/gps.h
  include/mrpt/ros1bridge/logging.h
  include/mrpt/ros1bridge/point_cloud.h
  include/mrpt/ros1bridge/imu.h
  include/mrpt/ros1bridge/laser_scan.h
  include/mrpt/ros1bridge/point_cloud2.h
  include/mrpt/ros1bridge/image.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
   mrpt::maps
   mrpt::obs
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)

#target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads) # for WorkerThreads

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${roscpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
if (tf2_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${tf2_LIBRARIES})
endif()
if (TARGET tf2)
  target_link_libraries(${PROJECT_NAME} PUBLIC tf2)
endif()

target_include_directories(${PROJECT_NAME}
  SYSTEM
  PUBLIC
    ${tf2_INCLUDE_DIRS}
  PRIVATE
    ${cv_bridge_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${ROS_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME} PRIVATE IS_MRPT_ROS1BRIDGE)
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
