cmake_minimum_required(VERSION 2.6)
# Disable in-source builds --> This lines must be commented if i want to use the command make eclipse-project'
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
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#rosbuild_genmsg()
#rosbuild_gensrv()
# rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# target_link_libraries(${PROJECT_NAME} another_library)
# rosbuild_add_boost_directories()
# rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(binNodoTransformacionesSistemasReferencia src/nodoTransformacionesSistemasReferencia.cpp)
rosbuild_add_executable(binNodoEscuchador src/nodoEscuchador.cpp)
