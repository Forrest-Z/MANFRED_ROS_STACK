MRL_ADD_EXECUTABLE("AUTO" "ALL_FILES_RECURSE")

IF(JPEG_FOUND)
	TARGET_LINK_LIBRARIES(${EXE_NAME} ${JPEG_LIBRARIES})
ENDIF(JPEG_FOUND)

TARGET_LINK_LIBRARIES(${EXE_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

