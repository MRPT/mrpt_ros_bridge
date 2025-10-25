cmake_minimum_required(VERSION 3.10)
project(rosbag2rawlog)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin and required ROS 1 components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  rosbag
  mrpt_msgs
  mrpt_libros_bridge
)

# Find external dependencies
find_package(mrpt-apps REQUIRED)
find_package(mrpt-ros1bridge REQUIRED)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    mrpt_msgs
    mrpt_libros_bridge
  #DEPENDS
    #mrpt-apps
)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall)
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

######################
## rosbag2rawlog    ##
######################

add_executable(${PROJECT_NAME}
  src/rosbag2rawlog.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mrpt::containers
  mrpt::ros1bridge
)

# ===================================================================================================
# Helper macros for version encoding (unchanged)
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

# Encode cv_bridge version for preprocessor use
mrpt_version_to_hex(cv_bridge_VERSION cv_bridge_VERSION_HEX)
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
# ===================================================================================================

# Installation rules
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# === Optional: add tests (roslint, etc.) ===
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add tests here if needed
endif()
