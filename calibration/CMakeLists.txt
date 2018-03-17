
include_directories( ${OpenCV_INCLUDE_DIRS} ../engine/include/ )

add_library( calibration
			 calibrationPatternUtils.cpp
			 calibrationBaseDetector.cpp
             concentric/calibrationPatternConcentricImpl.cpp
             circleGridSymmetric/calibrationPatternCircleGridSymmetricImpl.cpp
             circleGridAsymmetric/calibrationPatternCircleGridAsymmetricImpl.cpp
             chessboard/calibrationPatternChessboardImpl.cpp )

target_link_libraries( calibration ${OpenCV_LIBS} calibcvEngine ${CMAKE_THREAD_LIBS_INIT} )