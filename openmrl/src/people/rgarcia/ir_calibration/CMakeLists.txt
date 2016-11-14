MRL_ADD_EXECUTABLE("AUTO" "ALL_FILES")

# "m" seems to be needed when using a c program with math.h
TARGET_LINK_LIBRARIES(${EXE_NAME})

# ===== Adding a third-party library: =====
# We are going to use lmfit library, so we put a name to it: lmfit_lib
#add_library( lmfit_lib STATIC IMPORTED )
# IMPORTED tells CMake that this library is pre-built, and that there are therefore no sources to specify

#Once you've defined the library, you then have to tell CMake where the prebuilt library is located. This can be done by setting the 'IMPORTED_LOCATION' property on your new library:
#set_property( TARGET lmfit_lib PROPERTY IMPORTED_LOCATION /usr/local/lib/liblmmin.a)

#If the library needs some includes, you can specify them using 
#INCLUDE_DIRECTORIES(/usr/local/include)

#set(CMAKE_CXX_FLAGS "-g -Wall")

# If you need some link dir:

#find_package(serial REQUIRED)
#include_directories(${serial_INCLUDE_DIRS})
#TARGET_LINK_LIBRARIES(${EXE_NAME} ${serial_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
# If you need to link some other external library:
#LINK_DIRECTORIES(/usr/local/lib)


