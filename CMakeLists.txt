
cmake_minimum_required( VERSION 3.5.0 )

project( calibcv )

if ( WIN32 )
	message( "CONFIGURING FOR WINDOWS" )

	set( "OpenCV_DIR" "C:/opencv/build" )
	find_package( OpenCV REQUIRED )
	find_package( Threads REQUIRED )

	add_subdirectory( engine )
	add_subdirectory( tests )

else()

	find_package( OpenCV REQUIRED )
	find_package( Threads REQUIRED )

	add_subdirectory( ext )
	add_subdirectory( engine )
	add_subdirectory( tests )
endif()


