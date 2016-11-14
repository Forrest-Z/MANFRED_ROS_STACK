cmake_minimum_required(VERSION 2.6)

# Disable in-source builds
# IF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

#     MESSAGE(FATAL_ERROR "In-source builds are not allowed.")

# ENDIF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#rosbuild_genmsg()
rosbuild_gensrv()
# rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# target_link_libraries(${PROJECT_NAME} another_library)
# rosbuild_add_boost_directories()
# rosbuild_link_boost(${PROJECT_NAME} thread)
# rosbuild_add_executable(example examples/example.cpp)
# target_link_libraries(example ${PROJECT_NAME})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
rosbuild_add_executable(binNodoSicks3000 src/nodoSicks3000.cpp src/sicks3000.cpp src/serialIO.cpp)
rosbuild_add_executable(binNodoSicks3000Bag src/nodoSicks3000Bag.cpp src/sicks3000.cpp src/serialIO.cpp)

