
message( "Building tests" )

add_executable( testThresholding.out
                testThresholding.cpp )

target_link_libraries( testThresholding.out 
                       ${OpenCV_LIBS} )