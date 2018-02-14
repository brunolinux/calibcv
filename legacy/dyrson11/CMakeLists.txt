
message( "Configuring dyrson11 tests" )

# include_directories( ${OpenCV_INCLUDE_DIRS} /usr/include/ )
# link_directories( /usr/lib )

# message( "cv libs: ${OpenCV_LIBS}" )

add_library( calibcv2
             CUtils.cpp
             CTuning.cpp
             CDetectionPanel.cpp
             CDetectionPipeline.cpp )

add_executable( testCapture_d.out
                testCapture.cpp )

target_link_libraries( testCapture_d.out
                       ${OpenCV_LIBS}
                       calibcv2 )

add_executable( testThresholding_d.out
                testThresholding.cpp )

target_link_libraries( testThresholding_d.out
                       ${OpenCV_LIBS}
                       calibcv2 )


add_executable( testContours_d.out
                testContours.cpp )

target_link_libraries( testContours_d.out
                       ${OpenCV_LIBS}
                       calibcv2 )
