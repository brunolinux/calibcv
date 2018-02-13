
cmake_minimum_required( VERSION 3.5.0 )

project( calibcv )

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
#find_package( CUDA ) # TODO: Add checks for cuda support

# add_subdirectory( wpumacay )
# add_subdirectory( dyrson11 )
add_subdirectory( engine )
add_subdirectory( tests )
