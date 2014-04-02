cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
if(DEFINED ENV{JENKINS_URL})
    include(/usr/local/share/cmake-pclint/pc_lint.cmake)
endif(DEFINED ENV{JENKINS_URL})

#rosbuild_make_distribution(0.1.0)

rosbuild_init()
set(WEBOTS_HOME $ENV{WEBOTS_HOME})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CONTROLLER_SOURCES
	src/main.cpp
	src/world.cpp
	src/youbot.cpp
	src/config.cpp
	src/hardware.cpp
	src/feedback_publisher.cpp
	src/sensor_laserscanner.cpp
	src/sensor_kinect.cpp
	src/sensor_camera.cpp
	src/actuator_base.cpp
	src/actuator_arm.cpp
	includes/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
	includes/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
	includes/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
	includes/base-kinematic/WheeledBaseKinematic.cpp
)
#
set(CMAKE_CXX__FLAGS -O3)

add_definitions(-Wall -DLINUX)
include_directories(${WEBOTS_HOME}/include/controller/cpp)
include_directories(${WEBOTS_HOME}/include/controller/c)
include_directories(${PROJECT_SOURCE_DIR}/includes)

rosbuild_add_executable(youbot ${CONTROLLER_SOURCES})
find_library(CPP_CONTROLLER_LIBRARY CppController ${WEBOTS_HOME}/lib)
find_library(C_CONTROLLER_LIBRARY Controller ${WEBOTS_HOME}/lib)
find_library(XML_LIBRARY tinyxml)

rosbuild_add_boost_directories()

target_link_libraries(youbot ${C_CONTROLLER_LIBRARY} ${CPP_CONTROLLER_LIBRARY} ${XML_LIBRARY})

if(COMMAND add_pc_lint)
  add_pc_lint(youbot ${CONTROLLER_SOURCES})
endif(COMMAND add_pc_lint)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)
