cmake_minimum_required(VERSION 2.4.6)
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
# set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../ficherosFindCMake  CACHE PATH "CMAKE_MODULE_PATH MODIFIED")
find_package(Pmac REQUIRED)
include_directories(${Pmac_INCLUDE_DIRS})
# include_directories(${PROJECT_SOURCE_DIR}/include)
rosbuild_add_boost_directories()
rosbuild_add_executable(binNodoTeleopBaseTecladoPub src/nodoTeleopBaseTecladoPub.cpp)
rosbuild_link_boost(binNodoTeleopBaseTecladoPub thread)

rosbuild_add_executable(binNodoTeleopBaseTecladoSus src/nodoTeleopBaseTecladoSus.cpp)
target_link_libraries(binNodoTeleopBaseTecladoSus ${Pmac_LIBRARIES})
rosbuild_link_boost(binNodoTeleopBaseTecladoSus thread)
