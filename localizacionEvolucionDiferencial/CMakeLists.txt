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
set(ROS_BUILD_TYPE MinSizeRel)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
find_package(ImageMagick COMPONENTS Magick++ MagickWand MagickCore animate compare composite conjure convert)
find_package(ImageMagick COMPONENTS display identify import mogrify montage stream)
include_directories(${ImageMagick_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
#rosbuild_genmsg()
#rosbuild_gensrv()
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(binNodoLocalizacionEvolucionDiferencial src/nodoLocalizacionEvolucionDiferencial.cpp src/localizacionEvolucionDiferencial.cpp src/mapa.cpp)
TARGET_LINK_LIBRARIES(binNodoLocalizacionEvolucionDiferencial ${ImageMagick_LIBRARIES})
rosbuild_add_executable(binNodoLocalizacionEvolucionDiferencialBag src/nodoLocalizacionEvolucionDiferencialBag.cpp src/localizacionEvolucionDiferencial.cpp src/mapa.cpp)
TARGET_LINK_LIBRARIES(binNodoLocalizacionEvolucionDiferencialBag ${ImageMagick_LIBRARIES})
