
include_directories( ${OpenCV_INCLUDE_DIRS} ../engine/include/ )

add_library( calibration
			 calibrationPatternConcentricImpl.cpp
			 calibrationPatternConcentricUtils.cpp )

target_link_libraries( calibration ${OpenCV_LIBS} calibcvEngine ${CMAKE_THREAD_LIBS_INIT} )