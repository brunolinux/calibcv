
include_directories( ${OpenCV_INCLUDE_DIRS} ../engine/include/ )

add_library( calibration
			 calibrationPatternUtils.cpp
             calibrationBaseDetector.cpp
			 calibrationInterface.cpp
             concentric/calibrationPatternConcentric.cpp
             concentric/calibrationPatternConcentricImpl.cpp
             circleGridSymmetric/calibrationPatternCircleGridSymmetric.cpp
             circleGridSymmetric/calibrationPatternCircleGridSymmetricImpl.cpp
             circleGridAsymmetric/calibrationPatternCircleGridAsymmetric.cpp
             circleGridAsymmetric/calibrationPatternCircleGridAsymmetricImpl.cpp
             chessboard/calibrationPatternChessboard.cpp
             chessboard/calibrationPatternChessboardImpl.cpp )

target_link_libraries( calibration ${OpenCV_LIBS} calibcvEngine ${CMAKE_THREAD_LIBS_INIT} )