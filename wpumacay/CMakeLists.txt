
message( "Configuring tests" )

# include_directories( ${OpenCV_INCLUDE_DIRS} /usr/include/ )
# link_directories( /usr/lib )

# message( "cv libs: ${OpenCV_LIBS}" )

add_library( calibcv
             CUtils.cpp
             CTuning.cpp )

add_executable( testCapture.out
                testCapture.cpp )

target_link_libraries( testCapture.out
                       ${OpenCV_LIBS}
                       calibcv )

add_executable( testThresholding.out
                testThresholding.cpp )

target_link_libraries( testThresholding.out
                       ${OpenCV_LIBS}
                       calibcv )
