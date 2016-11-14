MRL_ADD_EXECUTABLE("AUTO" "ALL_FILES")

#set(CMAKE_CXX_FLAGS "-g -Wall")

# If you need some include dir:
#INCLUDE_DIRECTORIES("/usr/include/serialport/")

# If you need some link dir:
#LINK_DIRECTORIES(serialPort)

#find_package(serial REQUIRED)
#include_directories(${serial_INCLUDE_DIRS})
#TARGET_LINK_LIBRARIES(${EXE_NAME} ${serial_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

# If you need to link some other external library:
TARGET_LINK_LIBRARIES(${EXE_NAME} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

