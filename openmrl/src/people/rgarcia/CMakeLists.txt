IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/COMPILE_IT")
	MESSAGE(STATUS "Enabling compilation of Ruy Garcia personal stuff (because of ${CMAKE_CURRENT_SOURCE_DIR}/COMPILE_IT)")
	ADD_SUBDIRECTORY(ir_calibration)
ENDIF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/COMPILE_IT")


