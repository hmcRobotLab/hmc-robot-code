cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#include_directories(/usr/local/openni)

#find_path(OPENNI_INCLUDE_DIRS
#    NAMES XnOS.h
#    PATH_SUFFIXES ni)
#if("${OPENNI_INCLUDE_DIRS}" STREQUAL "OPENNI_INCLUDE_DIRS-NOTFOUND")
#    set(OPENNI_EXAMPLE_MESSAGE "Disabled - XnOS.h not found" PARENT_SCOPE)
#else()
#    find_library(OPENNI_LIBS OpenNI)
#    if("${OPENNI_LIBS}" STREQUAL "OPENNI_LIBS-NOTFOUND")
#        set(OPENNI_EXAMPLE_MESSAGE "Disabled - OpenNI library not found" PARENT_SCOPE)
#    else()
#        set(OPENNI_EXAMPLE_MESSAGE "Enabled" PARENT_SCOPE)
#    endif()
#endif()

include_directories(${OPENNI_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

add_subdirectory(libfovis)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
include_directories(libfovis/build/lib libfovis/libfovis)
#rosbuild_add_executable(odomPubRos src/odomPubRos.cpp src/data_capture_ros.cpp)
#target_link_libraries(odomPubRos ${OPENNI_LIBS} fovis)
rosbuild_add_executable(odomPubRos src/odomPubRos.cpp src/data_capture_ros.cpp)
target_link_libraries(odomPubRos ${OPENNI_LIBS} fovis)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
