cmake_minimum_required(VERSION 2.4.3)

# Disable in-source builds
#IF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
#     MESSAGE(FATAL_ERROR "In-source builds are not allowed.")
#ENDIF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
message(STATUS "ROS_PACKAGE_PATH: " $ENV{ROS_PACKAGE_PATH})
message(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_BUILD_DIR: " ${CMAKE_BINARY_DIR})
message(STATUS "PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})
#find_package(ImageMagick COMPONENTS Magick++ MagickWand MagickCore animate compare composite conjure convert)
#find_package(ImageMagick COMPONENTS display identify import mogrify montage stream)
#message(STATUS "ImageMagick_INCLUDE_DIRS: " ${ImageMagick_INCLUDE_DIRS})
#message(STATUS "ImageMagick_LIBRARIES: " ${ImageMagick_LIBRARIES})
#include_directories(${ImageMagick_INCLUDE_DIRS})
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../ficherosFindCMake CACHE PATH "CMAKE_MODULE_PATH")
find_package(OpenMRL REQUIRED)
include_directories(${OpenMRL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
# Uncomment if you have defined messages
#rosbuild_genmsg()
# Uncomment if you have defined services
#rosbuild_gensrv()
rosbuild_add_executable(binNodoFastMarching src/nodoFastMarching.cpp)
target_link_libraries(binNodoFastMarching ${OpenMRL_LIBRARIES})
# Common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
