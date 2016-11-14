cmake_minimum_required(VERSION 2.6)
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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#rosbuild_genmsg()
#rosbuild_gensrv()
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_FLAGS "-g -Wall")
set(LIBRARY_NAME pmac)
message(STATUS "LIBRARY_NAME: "  ${LIBRARY_NAME})
add_library(${LIBRARY_NAME} STATIC src/pmac.cpp)
#add_executable(binData src/data.cpp)
#target_link_libraries(binData ${LIBRARY_NAME})
add_executable(binComandos src/comandos.cpp)
target_link_libraries(binComandos ${LIBRARY_NAME})
add_executable(binTerminal src/terminal.cpp)
target_link_libraries(binTerminal ${LIBRARY_NAME})
