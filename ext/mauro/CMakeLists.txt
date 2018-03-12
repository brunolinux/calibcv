
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

find_package( OpenGL REQUIRED )
find_package( Threads REQUIRED )

set( MAURO_INCLUDE_DIRECTORIES
	 lib/include/
	 "${CMAKE_SOURCE_DIR}/ext/glfw/include"
	 "${CMAKE_BINARY_DIR}/ext/glad/include" )

# message( "mauroInc: ${MAURO_INCLUDE_DIRECTORIES}" )

include_directories( ${MAURO_INCLUDE_DIRECTORIES} )

## Configure library *****************************************

FILE( GLOB_RECURSE MauroSources lib/src *.cpp )

add_library( mauro
			 ${MauroSources} )

target_link_libraries( mauro
					   glfw glad
					   ${OPENGL_LIBRARIES}
					   ${CMAKE_THREAD_LIBS_INIT} )


## Configure tests *******************************************

function( MauroFnBuildTest pSources pExecutableId )

    add_executable( ${pExecutableId}
                    ${pSources} )

    target_link_libraries( ${pExecutableId}
                           mauro )
    
endfunction()

set( MauroTestsSources
     tests/testWindows.cpp )

message( "mauro tests: ${MauroTestsSources}" )

foreach( testFile ${MauroTestsSources} )

    string( REPLACE ".cpp" ""  executableId ${testFile} )
    get_filename_component( execId ${executableId} NAME )

    # message( "fileId: ${testFile}" )
    # message( "fullPathExecutableID: ${executableId}" )
    # message( "executableId: ${execId}")

    MauroFnBuildTest( ${testFile} ${execId} )

endforeach( testFile )