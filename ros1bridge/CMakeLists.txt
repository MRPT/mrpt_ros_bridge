cmake_minimum_required(VERSION 3.10)
project(mrpt_libros_bridge)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect ROS 1 environment
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  stereo_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
)

# Find MRPT dependency (from mrpt_libapps)
find_package(mrpt-apps REQUIRED)

# Define catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    nav_msgs
    stereo_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    cv_bridge
  #DEPENDS mrpt-apps
)

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
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${catkin_INCLUDE_DIRS}
    ${mrpt-apps_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mrpt::apps
)

# Installation rules
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Export the targets to a CMake package
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE mrpt::
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/mrpt-ros1bridge
)

# === Tests ===
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_${PROJECT_NAME}
    src/map_unittest.cpp
    src/pointcloud2_unittest.cpp
    src/pose_unittest.cpp
    src/time_unittest.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
  endif()
endif()

# === MRPT version macros (unchanged) ===
function(pad_string output str padchar length)
  string(LENGTH "${str}" _strlen)
  math(EXPR _strlen "${length} - ${_strlen}")
  if(_strlen GREATER 0)
    string(REPEAT ${padchar} ${_strlen} _pad)
    string(PREPEND str ${_pad})
  endif()
  set(${output} "${str}" PARENT_SCOPE)
endfunction()

macro(mrpt_version_to_hex VER TARGET_VAR_NAME)
  string(REGEX MATCHALL "[0-9]+" __parts "${${VER}}")
  if(__parts)
    list(GET __parts 0 __VERSION_NUMBER_MAJOR)
    list(GET __parts 1 __VERSION_NUMBER_MINOR)
    list(GET __parts 2 __VERSION_NUMBER_PATCH)
    pad_string(__VERSION_NUMBER_MAJOR ${__VERSION_NUMBER_MAJOR} "0" 2)
    pad_string(__VERSION_NUMBER_MINOR ${__VERSION_NUMBER_MINOR} "0" 2)
    pad_string(__VERSION_NUMBER_PATCH ${__VERSION_NUMBER_PATCH} "0" 2)
    set(${TARGET_VAR_NAME} "0x${__VERSION_NUMBER_MAJOR}${__VERSION_NUMBER_MINOR}${__VERSION_NUMBER_PATCH}")
  else()
    set(${TARGET_VAR_NAME} "0x000")
  endif()
endmacro()

# Convert package versions to hex so they can be used in preprocessor
mrpt_version_to_hex(cv_bridge_VERSION cv_bridge_VERSION_HEX)
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})

# === Export a standard CMake config file (like ROS 2 ament does) ===
include(CMakePackageConfigHelpers)

# Create an alias target for compatibility
add_library(mrpt::ros1bridge ALIAS ${PROJECT_NAME})

# Extract version number from package.xml
file(READ ${CMAKE_SOURCE_DIR}/package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MRPT_ROS1BRIDGE_VERSION ${CMAKE_MATCH_1})
message(STATUS "MRPT_ROS1BRIDGE_VERSION: ${MRPT_ROS1BRIDGE_VERSION}")

# Generate the config file
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mrpt-ros1bridge-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros1bridge-config.cmake"
  INSTALL_DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/mrpt-ros1bridge"
)

# Generate version file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros1bridge-config-version.cmake"
  VERSION ${MRPT_ROS1BRIDGE_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install both config + version files
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros1bridge-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mrpt-ros1bridge-config-version.cmake"
  DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/mrpt-ros1bridge"
)