
cmake_minimum_required( VERSION 3.5.0 )

project( calibcv )

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )

add_subdirectory( ext )
add_subdirectory( engine )
add_subdirectory( tests )
