###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done: 
#  
#  OpenCV_FOUND
#  OpenCV_LIBS
#  OpenCV_INCLUDE_DIR
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#
# Deprecated variable are used to maintain backward compatibility with
# the script of Jan Woetzel (2006/09): www.mip.informatik.uni-kiel.de/~jw
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
# 
## 3: Version
#
# 2011/06/15 Luca Marchetti, Added support for OpenRDK.
# 2010/04/07 Benoit Rat, Correct a bug when OpenCVConfig.cmake is not found.
# 2010/03/24 Benoit Rat, Add compatibility for when OpenCVConfig.cmake is not found.
# 2010/03/22 Benoit Rat, Creation of the script.
#
#
# tested with:
# - OpenCV 2.2:  GCC4
# - OpenCV 2.1:  MinGW, MSVC2008
# - OpenCV 2.0:  MinGW, MSVC2008, GCC4
#
#
## 4: Licence:
#
# LGPL 2.1 : GNU Lesser General Public License Usage
# Alternatively, this file may be used under the terms of the GNU Lesser

# General Public License version 2.1 as published by the Free Software
# Foundation and appearing in the file LICENSE.LGPL included in the
# packaging of this file.  Please review the following information to
# ensure the GNU Lesser General Public License version 2.1 requirements
# will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
# 
#----------------------------------------------------------
IF("${CMAKE_CROSSCOMPILING}" STREQUAL TRUE)
	IF ("${OpenRDK_ARCH}" STREQUAL "geode" )
		# be backward compatible:
		INCLUDE(${OE_CMAKE_MODULE_PATH}/OPENCVConfig.cmake)
		SET(OpenCV_FOUND ${OPENCV_FOUND})
		SET(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIR}/opencv)
		SET(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})
	ELSEIF ("${OpenRDK_ARCH}" STREQUAL "arm9" )

		FIND_PATH(OpenCV_INCLUDE_DIRS cv.h
			${ARM9_CROSS_DIR}/include
			${ARM9_CROSS_ADDONS_DIR}/include
			${ARM9_CROSS_DIR}/include/opencv
			${ARM9_CROSS_ADDONS_DIR}/include/opencv
			NO_CMAKE_SYSTEM_PATH
			)
		FIND_PATH(OpenCV_LINK_DIRECTORIES libcv.so
			${ARM9_CROSS_DIR}/lib
			${ARM9_CROSS_ADDONS_DIR}/lib
			NO_CMAKE_SYSTEM_PATH
			)
		SET(OpenCV_LIBRARIES "-lcxcore -lcv -lcvaux -lml")

		IF(OpenCV_INCLUDE_DIRS AND OpenCV_LINK_DIRECTORIES)
			SET(OpenCV_FOUND ON)
			SET(OPENCV_LIBRARIES   ${OpenCV_LIBRARIES} )
			SET(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS} )
			SET(OPENCV_LINK_DIRECTORIES ${OpenCV_LINK_DIRECTORIES})
			SET(OPENCV_FOUND   ${OpenCV_FOUND})


			#     STRING(REGEX REPLACE ".*/([^/]*)$" "\\1" OPENCV_LIBRARIES ${OPENCV_LINK_DIRECTORIES})
			#STRING(REGEX REPLACE "/[^/]*$" "" OPENCV_LINK_DIRECTORIES ${OPENCV_LINK_DIRECTORIES})
		ENDIF(OpenCV_INCLUDE_DIRS AND OpenCV_LINK_DIRECTORIES)
	ELSE ("${OpenRDK_ARCH}" STREQUAL "geode" )
		MESSAGE(STATUS "OpenCV support is not available for unknown architecture")
	ENDIF ("${OpenRDK_ARCH}" STREQUAL "geode" )

ELSE("${CMAKE_CROSSCOMPILING}" STREQUAL TRUE)

	find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV")

	if (NOT "x${ROOT_OpenCV}" STREQUAL "x")
		set(OpenCV_DIR ${ROOT_OpenCV})
	endif (NOT "x${ROOT_OpenCV}" STREQUAL "x")

	##====================================================
	## Find OpenCV libraries
	##----------------------------------------------------
	#When its possible to use the Config script use it.
	if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

		## Include the standard CMake script
		include("${OpenCV_DIR}/OpenCVConfig.cmake")

		## Search for a specific version
		set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

	else(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

		#Otherwise it try to guess it.
		# looking for opencv2 first
		set(OpenCV_INCLUDE_DIR OpenCV_INCLUDE_DIR-NOTFOUND CACHE INTERNAL "" FORCE)

		find_path(OpenCV_INCLUDE_DIR "opencv2/core/version.hpp" HINTS "${OpenCV_DIR}" PATH_SUFFIXES "include"  DOC "" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
		# looking for opencv
		find_path(OpenCV_INCLUDE_DIR "opencv/cvver.h" HINTS "${OpenCV_DIR}" "/usr/local" "/usr" PATH_SUFFIXES "include" DOC "" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)

		# getting version file name
		set(OpenCV_VERSION_FILE OpenCV_VERSION_FILE-NOTFOUND CACHE INTERNAL "" FORCE)
		find_file(OpenCV_VERSION_FILE opencv2/core/version.hpp HINTS ${OpenCV_INCLUDE_DIR} PATH_SUFFIXES "include" DOC ""  NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
		find_file(OpenCV_VERSION_FILE opencv/cvver.h HINTS ${OpenCV_INCLUDE_DIR} PATH_SUFFIXES "include" DOC "" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)

		if ("${OpenCV_INCLUDE_DIR}" STREQUAL "OpenCV_INCLUDE_DIR-NOTFOUND"
				OR
				"${OpenCV_VERSION_FILE}" STREQUAL "OpenCV_VERSION_FILE-NOTFOUND")
			set(OpenCV_FOUND FALSE)
		else("${OpenCV_INCLUDE_DIR}" STREQUAL "OpenCV_INCLUDE_DIR-NOTFOUND"
				OR
				"${OpenCV_VERSION_FILE}" STREQUAL "OpenCV_VERSION_FILE-NOTFOUND")

			if(EXISTS ${OpenCV_INCLUDE_DIR})
				#Find OpenCV version by looking at core/version.hpp or cvver.h
				file(STRINGS ${OpenCV_VERSION_FILE} OpenCV_VERSIONS_TMP REGEX "^#define CV_[A-Z]+_VERSION[ \t]+[0-9]+$")
				string(REGEX REPLACE ".*#define CV_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MAJOR ${OpenCV_VERSIONS_TMP})
				string(REGEX REPLACE ".*#define CV_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MINOR ${OpenCV_VERSIONS_TMP})
				string(REGEX REPLACE ".*#define CV_SUBMINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_PATCH ${OpenCV_VERSIONS_TMP})
				set(OpenCV_VERSION ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH})
			endif(EXISTS  ${OpenCV_INCLUDE_DIR})

			if (NOT "x${ROOT_OpenCV}" STREQUAL "x")
				set(ENV{PKG_CONFIG_PATH} "${ROOT_OpenCV}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
			endif (NOT "x${ROOT_OpenCV}" STREQUAL "x")
			if(PKG_CONFIG_FOUND)
				# libs
				execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} opencv --libs-only-l OUTPUT_VARIABLE OpenCV_LIBRARIES_tmp OUTPUT_STRIP_TRAILING_WHITESPACE)
				string(REGEX REPLACE "-l([!\ ]*)" "\\1;" OpenCV_LIBRARIES_tmp ${OpenCV_LIBRARIES_tmp})
				string(REPLACE " " "" OpenCV_LIBRARIES_tmp "${OpenCV_LIBRARIES_tmp}")
				foreach(__lib ${OpenCV_LIBRARIES_tmp})
					list(APPEND OpenCV_LIBRARIES "${__lib}")
				endforeach(__lib ${OpenCV_LIBRARIES_tmp})

				# link-dir
				execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} opencv --libs-only-L OUTPUT_VARIABLE OpenCV_LINK_DIRECTORIES_tmp OUTPUT_STRIP_TRAILING_WHITESPACE)
				if (NOT "x${OpenCV_LINK_DIRECTORIES_tmp}" STREQUAL "x")
					string(REGEX REPLACE "-L([!\ ]*)" "\\1;" OpenCV_LINK_DIRECTORIES_tmp ${OpenCV_LINK_DIRECTORIES_tmp})
					string(REPLACE " " "" OpenCV_LINK_DIRECTORIES_tmp "${OpenCV_LINK_DIRECTORIES_tmp}")
					foreach(__lib ${OpenCV_LINK_DIRECTORIES_tmp})
						list(APPEND OpenCV_LINK_DIRECTORIES "${__lib}")
					endforeach(__lib ${OpenCV_LINK_DIRECTORIES_tmp})
					string(REGEX REPLACE "-L" ";" OpenCV_LINK_DIRECTORIES ${OpenCV_LINK_DIRECTORIES_tmp})
				else (NOT "x${OpenCV_LINK_DIRECTORIES_tmp}" STREQUAL "x")
					# fallback to standard lib directories for empty output
					set(OpenCV_LINK_DIRECTORIES "/usr/lib")
				endif (NOT "x${OpenCV_LINK_DIRECTORIES_tmp}" STREQUAL "x")
			else(PKG_CONFIG_FOUND)
				# manually setting path
				if (${OpenCV_VERSION_MINOR} EQUAL 2)
					# for OpenCV 2.2.x
					set(OpenCV_LIBRARIES "opencv_cor;opencv_imgproc;opencv_highgui;opencv_ml;opencv_video;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_contrib;opencv_legacy;opencv_flann")
				else(${OpenCV_VERSION_MINOR} EQUAL 2)
					# for OpenCV 2.1.x
					set(OpenCV_LIBRARIES "ml;cvaux;highgui;cv;cxcore")
				endif(${OpenCV_VERSION_MINOR} EQUAL 2)
			endif(PKG_CONFIG_FOUND)

			# generating full-path libraries
			if (NOT "x${OpenCV_LINK_DIRECTORIES}" STREQUAL "x")
				foreach(lib ${OpenCV_LIBRARIES})
					set(__libfile __libfile-NOTFOUND CACHE INTERNAL "" FORCE)
					find_library(__libfile NAMES ${lib} HINTS ${OpenCV_LINK_DIRECTORIES} NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
					if (NOT "${__libfile}" STREQUAL "__libfile-NOTFOUND")
						list(APPEND __opencv_flib ${__libfile})
					elseif (NOT "${__libfile}" STREQUAL "__libfile-NOTFOUND")
						error("OpenCV uses lib ${__libfile}, but I cannot find it in the OpenCV_LINK_DIRECTORIES variable (=${OpenCV_LINK_DIRECTORIES})")
					endif (NOT "${__libfile}" STREQUAL "__libfile-NOTFOUND")
				endforeach(lib ${OpenCV_LIBRARIES})
			endif (NOT "x${OpenCV_LINK_DIRECTORIES}" STREQUAL "x")

			set(OpenCV_LIBRARIES ${__opencv_flib})
			#
			# Logic selecting required libs and headers
			#
			if("x${OpenCV_LIBRARIES}" STREQUAL "x" AND NOT "x${OpenCV_INCLUDE_DIR}" STREQUAL "x")
				warning("OpenCV_INCLUDE_DIR found, but OpenCV_LIBRARIES is empty, did you forget to set PKG_CONFIG_PATH?")
			endif("x${OpenCV_LIBRARIES}" STREQUAL "x" AND NOT "x${OpenCV_INCLUDE_DIR}" STREQUAL "x")
			SET(OpenCV_FOUND ON)
		endif("${OpenCV_INCLUDE_DIR}" STREQUAL "OpenCV_INCLUDE_DIR-NOTFOUND"
			OR
			"${OpenCV_VERSION_FILE}" STREQUAL "OpenCV_VERSION_FILE-NOTFOUND")
	endif(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
endIF("${CMAKE_CROSSCOMPILING}" STREQUAL TRUE)

# display help message
IF(OpenCV_FOUND)
	verbose("Found OpenCV version ${OpenCV_VERSION}")
ENDIF(OpenCV_FOUND)

set(OpenCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
set(OpenCV_DIR ${OpenCV_DIR} CACHE INTERNAL "" FORCE)
set(OpenCV_VERSION_FILE ${OpenCV_VERSION_FILE} CACHE INTERNAL "" FORCE)
set(__libfile __libfile-NOTFOUND CACHE INTERNAL "" FORCE)
