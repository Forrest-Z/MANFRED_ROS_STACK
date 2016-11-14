# This is the main CMake configuration file, you should always do "cmake ." from here and not from subdirs

# Main project declaration
PROJECT(OpenMRL CXX C)

# Useful declaration for avoiding relative path
SET(OpenMRL_CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build_tools/cmake")
SET(OpenMRL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

INCLUDE(${OpenMRL_CMAKE_MODULE_PATH}/prettymessage.cmake)
INCLUDE(${OpenMRL_CMAKE_MODULE_PATH}/utils.cmake)


# Disable in-source builds
IF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
	MESSAGE(FATAL_ERROR "In-source builds are not allowed.")
ENDIF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")


# Verbose or not verbose
IF($ENV{VERBOSE})
	info("Verbose mode on")
ELSE($ENV{VERBOSE})
	info("Verbose mode off (type 'VERBOSE=1 cmake .' to enable it)")
ENDIF($ENV{VERBOSE})

# Needed by CMake versione 2.6 or later (Run "cmake --help-policy CMP0003" for more information.)"
# specifically, we are using full-path when possible for linking libraries
# this should be mandatory for packages found using ROOT directive
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Version
SET(OpenMRL_V_MAJOR 0)
SET(OpenMRL_V_MINOR 0)
SET(OpenMRL_V_PATCH 1)

# Check for SVN and get current revision
#INCLUDE(${OpenMRL_CMAKE_MODULE_PATH}/vcs-revision.cmake)
# Do not use revision information if I was unable to retrieve it
IF(NOT ${PROJECT_WC_REVISION} EQUAL 0)
	SET(OpenMRL_VERSION ${OpenMRL_V_MAJOR}.${OpenMRL_V_MINOR}.${OpenMRL_V_PATCH}.${PROJECT_WC_REVISION})
	SET(OpenMRL_V_REVISION ${PROJECT_WC_REVISION})
ELSE(NOT ${PROJECT_WC_REVISION} EQUAL 0)
	SET(OpenMRL_VERSION ${OpenMRL_V_MAJOR}.${OpenMRL_V_MINOR}.${OpenMRL_V_PATCH})
ENDIF(NOT ${PROJECT_WC_REVISION} EQUAL 0)


# Optional flags and configurable stuff in ccmake
# Replacement for manual.cmake
SET(COMPILATION_FLAGS "-W -Wall -O2 -ggdb" CACHE STRING "CMake compilation flags")
SET(CMAKE_BUILD_TYPE "Debug")

# Operating System specific behaviors and switches
INCLUDE(${OpenMRL_CMAKE_MODULE_PATH}/os.cmake)
# Find and configure switches for external libraries and applications
INCLUDE(${OpenMRL_CMAKE_MODULE_PATH}/deps.cmake)

# Creation of config.h
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/src/config.h.cmake" "${CMAKE_CURRENT_SOURCE_DIR}/src/config.h")


info("")
info("..:: Variables ::..")
info("OpenMRL_VERSION = ${OpenMRL_VERSION}")
info("OpenMRL_DIR=${OpenMRL_DIR}")

info("")

INCLUDE_DIRECTORIES("${OpenMRL_DIR}/src")

FILE(GLOB_RECURSE ALG_SRC "src/algorithms/*.cpp")
FILE(GLOB_RECURSE DATA_SRC "src/datatype/*.cpp")
FILE(GLOB_RECURSE THIRD-PARTY_SRC "src/third-party/*.cpp")
FILE(GLOB_RECURSE C_THIRD-PARTY_SRC "src/third-party/*.c")
FILE(GLOB_RECURSE DRIVERS_SRC "src/drivers/*.cpp")
FILE(GLOB_RECURSE SYSTEM_SRC "src/system/*.cpp")
FILE(GLOB_RECURSE SIM_SRC "src/sim/*.cpp")
FILE(GLOB_RECURSE MODELS_SRC "src/models/*.cpp")


SET(SOURCES ${ALG_SRC}
				${DATA_SRC}
				${DRIVERS_SRC}
				${SYSTEM_SRC}
				${SIM_SRC}
				${MODELS_SRC}
				${THIRD-PARTY_SRC}
				${C_THIRD-PARTY_SRC}
	)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

ADD_LIBRARY(mrl SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(mrl ${OPENGL_LIBRARIES} ${JPEG_LIBRARIES} m)


# Subdirectories
ADD_SUBDIRECTORY(src/apps)
ADD_SUBDIRECTORY(src/people)
