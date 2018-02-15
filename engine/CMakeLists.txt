
message( "Configuring pipeline" )

include_directories( ${OpenCV_INCLUDE_DIRS} include/ ../ext/felixcam/lib/include )

FILE( GLOB_RECURSE EngineSources src *.cpp )

# message( "EngineSources: ${EngineSources}" )

add_library( calibcvEngine
             ${EngineSources} )

target_link_libraries( calibcvEngine
                       ${OpenCV_LIBS}
                       felixcam )